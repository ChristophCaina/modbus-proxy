# -*- coding: utf-8 -*-
#
# This file is part of the modbus-proxy project
#
# Copyright (c) 2020-2021 Tiago Coutinho
# Distributed under the GPLv3 license. See LICENSE for more info.


import asyncio
import pathlib
import argparse
import struct
import warnings
import contextlib
import logging.config
from urllib.parse import urlparse

__version__ = "0.8.0"


DEFAULT_LOG_CONFIG = {
    "version": 1,
    "formatters": {
        "standard": {"format": "%(asctime)s %(levelname)8s %(name)s: %(message)s"}
    },
    "handlers": {
        "console": {"class": "logging.StreamHandler", "formatter": "standard"}
    },
    "root": {"handlers": ["console"], "level": "INFO"},
}

log = logging.getLogger("modbus-proxy")


def parse_url(url):
    if "://" not in url:
        url = f"tcp://{url}"
    result = urlparse(url)
    if not result.hostname:
        url = result.geturl().replace("://", "://0")
        result = urlparse(url)
    return result


# ---------------------------------------------------------------------------
# SunSpec register conversion helpers
# ---------------------------------------------------------------------------

def _decode_int16(data, offset):
    """Decode a signed 16-bit integer from Modbus register bytes."""
    return struct.unpack_from(">h", data, offset)[0]


def _encode_float32(value):
    """Encode a float32 value as two Modbus registers (4 bytes, big-endian)."""
    return struct.pack(">f", value)


def _decode_float32(data, offset):
    """Decode a float32 value from two Modbus registers (4 bytes, big-endian)."""
    return struct.unpack_from(">f", data, offset)[0]


def _encode_int16_sf(value):
    """Encode a scaled float as int16 + scale factor register pair.

    Finds the scale factor that preserves maximum precision while keeping
    the int16 value within [-32768, 32767].
    """
    if value == 0:
        return struct.pack(">hh", 0, 0)
    # Try from most precise (sf=-10) to least precise (sf=0)
    for sf in range(-10, 1):
        scaled = round(value * (10 ** -sf))
        if -32768 <= scaled <= 32767:
            return struct.pack(">hh", int(scaled), sf)
    # Fallback: clamp to int16 range with sf=0
    scaled = max(-32767, min(32767, int(value)))
    return struct.pack(">hh", scaled, 0)


class RegisterConversion:
    """
    Describes a single SunSpec register conversion rule.

    Supports:
      int16+sf  → float32   (e.g. SolarEdge → Bosch Energy Manager)
      float32   → int16+sf  (e.g. Fronius   → SolarEdge-compatible client)

    Config example (YAML):

      register_conversions:
        - address: 40083        # I_AC_Power (base-1 Modbus address)
          sf_address: 40084     # I_AC_Power_SF
          source_type: int16    # what the real device sends
          target_type: float32  # what this proxy should serve

        - address: 40206        # M_AC_Power (meter)
          sf_address: 40210
          source_type: int16
          target_type: float32
    """

    VALID_TYPES = {"int16", "float32"}

    def __init__(self, address, source_type, target_type, sf_address=None):
        if source_type not in self.VALID_TYPES:
            raise ValueError(f"source_type must be one of {self.VALID_TYPES}")
        if target_type not in self.VALID_TYPES:
            raise ValueError(f"target_type must be one of {self.VALID_TYPES}")
        if source_type == target_type:
            raise ValueError("source_type and target_type must differ")
        if source_type == "int16" and sf_address is None:
            raise ValueError("sf_address is required when source_type is int16")

        # Convert base-1 Modbus address to 0-based protocol offset
        self.address = address - 1        # 0-based register index
        self.sf_address = (sf_address - 1) if sf_address is not None else None
        self.source_type = source_type
        self.target_type = target_type

    @classmethod
    def from_config(cls, cfg):
        return cls(
            address=cfg["address"],
            source_type=cfg["source_type"],
            target_type=cfg["target_type"],
            sf_address=cfg.get("sf_address"),
        )

    def __repr__(self):
        return (
            f"RegisterConversion(address={self.address + 1}, "
            f"{self.source_type}→{self.target_type})"
        )


class SunSpecConverter:
    """
    Applies RegisterConversion rules to Modbus TCP reply payloads.

    The converter patches the register data in-place within the raw Modbus
    TCP frame so that the downstream client receives the converted values.

    Scale factor values are cached from previous replies so that conversions
    can be applied even when the SF register is not included in the current
    response window.
    """

    def __init__(self, conversions):
        self.conversions = conversions
        # Cache: {sf_register_0based: int16_sf_value}
        self._sf_cache = {}

    def _parse_read_response(self, reply):
        """
        Parse a Modbus TCP read holding registers response.

        Returns (start_address_0based, register_count, data_bytes) or None
        if the frame is not a valid read response (function code 0x03).
        """
        if len(reply) < 9:
            return None
        # Byte 7 = function code
        func_code = reply[7]
        if func_code != 0x03:
            return None

        # Byte 8 = byte count
        byte_count = reply[8]
        if len(reply) < 9 + byte_count:
            return None

        # Extract the register data bytes
        data = bytearray(reply[9:9 + byte_count])
        reg_count = byte_count // 2
        return data, reg_count

    def _parse_read_request(self, request):
        """
        Parse a Modbus TCP read holding registers request.

        Returns (start_address_0based, register_count) or None.
        """
        if len(request) < 12:
            return None
        func_code = request[7]
        if func_code != 0x03:
            return None
        start = int.from_bytes(request[8:10], "big")
        count = int.from_bytes(request[10:12], "big")
        return start, count

    def update_sf_cache(self, request, reply):
        """
        After receiving a reply, cache any scale factor register values
        that fall within the response window.
        """
        req = self._parse_read_request(request)
        res = self._parse_read_response(reply)
        if req is None or res is None:
            return

        start, count = req
        data, _ = res

        for conv in self.conversions:
            if conv.sf_address is None:
                continue
            if start <= conv.sf_address < start + count:
                offset = (conv.sf_address - start) * 2
                sf_value = _decode_int16(data, offset)
                self._sf_cache[conv.sf_address] = sf_value
                log.debug(
                    "Cached SF register %d = %d", conv.sf_address + 1, sf_value
                )

    def transform_reply(self, request, reply):
        """
        Apply conversions to a Modbus TCP reply.

        Returns the (possibly modified) reply bytes.
        """
        req = self._parse_read_request(request)
        res = self._parse_read_response(reply)
        if req is None or res is None:
            return reply

        start, count = req
        data, _ = res

        modified = False
        for conv in self.conversions:
            if not (start <= conv.address < start + count):
                continue  # register not in this response window

            offset = (conv.address - start) * 2

            if conv.source_type == "int16" and conv.target_type == "float32":
                # int16 + SF → float32
                # Value register must be in window; SF may come from cache
                sf = self._sf_cache.get(conv.sf_address)
                if sf is None:
                    if start <= conv.sf_address < start + count:
                        sf_offset = (conv.sf_address - start) * 2
                        sf = _decode_int16(data, sf_offset)
                        self._sf_cache[conv.sf_address] = sf
                    else:
                        log.debug(
                            "SF register %d not in window and not cached, "
                            "skipping conversion of register %d",
                            conv.sf_address + 1, conv.address + 1,
                        )
                        continue

                raw_int = _decode_int16(data, offset)
                float_val = raw_int * (10 ** sf)
                encoded = _encode_float32(float_val)

                # float32 occupies 2 registers (4 bytes) — same as int16+SF
                # We write the float into the value register slot and zero
                # the SF register slot (SF=0 is neutral for any reader that
                # still tries to apply it)
                data[offset:offset + 2] = encoded[0:2]
                if start <= conv.sf_address < start + count:
                    sf_offset = (conv.sf_address - start) * 2
                    data[sf_offset:sf_offset + 2] = encoded[2:4]

                log.debug(
                    "Converted register %d: int16(%d)*10^%d → float32(%.4f)",
                    conv.address + 1, raw_int, sf, float_val,
                )
                modified = True

            elif conv.source_type == "float32" and conv.target_type == "int16":
                # float32 → int16 + SF
                # float32 spans 2 registers; value reg + next reg
                if offset + 4 > len(data):
                    continue
                float_val = _decode_float32(data, offset)
                encoded = _encode_int16_sf(float_val)
                data[offset:offset + 4] = encoded
                log.debug(
                    "Converted register %d: float32(%.4f) → int16+SF",
                    conv.address + 1, float_val,
                )
                modified = True

        if not modified:
            return reply

        # Rebuild the reply with the patched data
        reply = bytearray(reply)
        reply[9:9 + len(data)] = data
        return bytes(reply)


class Connection:
    def __init__(self, name, reader, writer):
        self.name = name
        self.reader = reader
        self.writer = writer
        self.log = log.getChild(name)

    async def __aenter__(self):
        return self

    async def __aexit__(self, exc_type, exc_value, tb):
        await self.close()

    @property
    def opened(self):
        return (
            self.writer is not None
            and not self.writer.is_closing()
            and not self.reader.at_eof()
        )

    async def close(self):
        if self.writer is not None:
            self.log.info("closing connection...")
            try:
                self.writer.close()
                await self.writer.wait_closed()
            except Exception as error:
                self.log.info("failed to close: %r", error)
            else:
                self.log.info("connection closed")
            finally:
                self.reader = None
                self.writer = None

    async def _write(self, data):
        self.log.debug("sending %r", data)
        self.writer.write(data)
        await self.writer.drain()

    async def write(self, data):
        try:
            await self._write(data)
        except Exception as error:
            self.log.error("writting error: %r", error)
            await self.close()
            return False
        return True

    async def _read(self):
        """Read ModBus TCP message"""
        header = await self.reader.readexactly(6)
        size = int.from_bytes(header[4:], "big")
        reply = header + await self.reader.readexactly(size)
        self.log.debug("received %r", reply)
        return reply

    async def read(self):
        try:
            return await self._read()
        except asyncio.IncompleteReadError as error:
            if error.partial:
                self.log.error("reading error: %r", error)
            else:
                self.log.info("client closed connection")
            await self.close()
        except Exception as error:
            self.log.error("reading error: %r", error)
            await self.close()


class Client(Connection):
    def __init__(self, reader, writer):
        peer = writer.get_extra_info("peername")
        super().__init__(f"Client({peer[0]}:{peer[1]})", reader, writer)
        self.log.info("new client connection")


class ModBus(Connection):
    def __init__(self, config):
        modbus = config["modbus"]
        url = parse_url(modbus["url"])
        bind = parse_url(config["listen"]["bind"])
        super().__init__(f"ModBus({url.hostname}:{url.port})", None, None)
        self.host = bind.hostname
        self.port = 502 if bind.port is None else bind.port
        self.modbus_host = url.hostname
        self.modbus_port = url.port
        self.timeout = modbus.get("timeout", None)
        self.connection_time = modbus.get("connection_time", 0)
        self.unit_id_remapping = config.get("unit_id_remapping") or {}
        self.server = None
        self.lock = asyncio.Lock()

        # SunSpec register conversions (optional)
        conversions_cfg = config.get("register_conversions") or []
        conversions = [RegisterConversion.from_config(c) for c in conversions_cfg]
        self.converter = SunSpecConverter(conversions) if conversions else None
        if conversions:
            log.info(
                "SunSpec conversions enabled on port %d: %s",
                self.port, conversions,
            )

    @property
    def address(self):
        if self.server is not None:
            return self.server.sockets[0].getsockname()

    async def open(self):
        self.log.info("connecting to modbus...")
        self.reader, self.writer = await asyncio.open_connection(
            self.modbus_host, self.modbus_port
        )
        self.log.info("connected!")

    async def connect(self):
        if not self.opened:
            await asyncio.wait_for(self.open(), self.timeout)
            if self.connection_time > 0:
                self.log.info("delay after connect: %s", self.connection_time)
                await asyncio.sleep(self.connection_time)

    async def write_read(self, data, attempts=2):
        async with self.lock:
            for i in range(attempts):
                try:
                    await self.connect()
                    coro = self._write_read(data)
                    return await asyncio.wait_for(coro, self.timeout)
                except Exception as error:
                    self.log.error(
                        "write_read error [%s/%s]: %r", i + 1, attempts, error
                    )
                    await self.close()

    async def _write_read(self, data):
        await self._write(data)
        return await self._read()

    def _transform_request(self, request):
        uid = request[6]
        new_uid = self.unit_id_remapping.setdefault(uid, uid)
        if uid != new_uid:
            request = bytearray(request)
            request[6] = new_uid
            self.log.debug("remapping unit ID %s to %s in request", uid, new_uid)
        return request

    def _transform_reply(self, request, reply):
        # Unit ID remapping (inverse)
        uid = reply[6]
        inverse_unit_id_map = {v: k for k, v in self.unit_id_remapping.items()}
        new_uid = inverse_unit_id_map.setdefault(uid, uid)
        if uid != new_uid:
            reply = bytearray(reply)
            reply[6] = new_uid
            self.log.debug("remapping unit ID %s to %s in reply", uid, new_uid)
            reply = bytes(reply)

        # SunSpec register conversion
        if self.converter is not None:
            self.converter.update_sf_cache(request, reply)
            reply = self.converter.transform_reply(request, reply)

        return reply

    async def handle_client(self, reader, writer):
        async with Client(reader, writer) as client:
            while True:
                request = await client.read()
                if not request:
                    break
                transformed_request = self._transform_request(request)
                reply = await self.write_read(transformed_request)
                if not reply:
                    break
                result = await client.write(self._transform_reply(request, reply))
                if not result:
                    break

    async def start(self):
        self.server = await asyncio.start_server(
            self.handle_client, self.host, self.port, start_serving=True
        )

    async def stop(self):
        if self.server is not None:
            self.server.close()
            await self.server.wait_closed()
        await self.close()

    async def serve_forever(self):
        if self.server is None:
            await self.start()
        async with self.server:
            self.log.info("Ready to accept requests on %s:%d", self.host, self.port)
            await self.server.serve_forever()


def load_config(file_name):
    file_name = pathlib.Path(file_name)
    ext = file_name.suffix
    if ext.endswith("toml"):
        from toml import load
    elif ext.endswith("yml") or ext.endswith("yaml"):
        import yaml

        def load(fobj):
            return yaml.load(fobj, Loader=yaml.Loader)

    elif ext.endswith("json"):
        from json import load
    else:
        raise NotImplementedError
    with open(file_name) as fobj:
        return load(fobj)


def prepare_log(config):
    cfg = config.get("logging")
    if not cfg:
        cfg = DEFAULT_LOG_CONFIG
    if cfg:
        cfg.setdefault("version", 1)
        cfg.setdefault("disable_existing_loggers", False)
        logging.config.dictConfig(cfg)
    warnings.simplefilter("always", DeprecationWarning)
    logging.captureWarnings(True)
    return log


def parse_args(args=None):
    parser = argparse.ArgumentParser(
        description="ModBus proxy",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "-c", "--config-file", default=None, type=str, help="config file"
    )
    parser.add_argument("-b", "--bind", default=None, type=str, help="listen address")
    parser.add_argument(
        "--modbus",
        default=None,
        type=str,
        help="modbus device address (ex: tcp://plc.acme.org:502)",
    )
    parser.add_argument(
        "--modbus-connection-time",
        type=float,
        default=0,
        help="delay after establishing connection with modbus before first request",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10,
        help="modbus connection and request timeout in seconds",
    )
    options = parser.parse_args(args=args)

    if not options.config_file and not options.modbus:
        parser.exit(1, "must give a config-file or/and a --modbus")
    return options


def create_config(args):
    if args.config_file is None:
        assert args.modbus
    config = load_config(args.config_file) if args.config_file else {}
    prepare_log(config)
    log.info("Starting...")
    devices = config.setdefault("devices", [])
    if args.modbus:
        listen = {"bind": ":502" if args.bind is None else args.bind}
        devices.append(
            {
                "modbus": {
                    "url": args.modbus,
                    "timeout": args.timeout,
                    "connection_time": args.modbus_connection_time,
                },
                "listen": listen,
            }
        )
    return config


def create_bridges(config):
    return [ModBus(cfg) for cfg in config["devices"]]


async def start_bridges(bridges):
    coros = [bridge.start() for bridge in bridges]
    await asyncio.gather(*coros)


async def run_bridges(bridges, ready=None):
    async with contextlib.AsyncExitStack() as stack:
        coros = [stack.enter_async_context(bridge) for bridge in bridges]
        await asyncio.gather(*coros)
        await start_bridges(bridges)
        if ready is not None:
            ready.set(bridges)
        coros = [bridge.serve_forever() for bridge in bridges]
        await asyncio.gather(*coros)


async def run(args=None, ready=None):
    args = parse_args(args)
    config = create_config(args)
    bridges = create_bridges(config)
    await run_bridges(bridges, ready=ready)


def main():
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        log.warning("Ctrl-C pressed. Bailing out!")


if __name__ == "__main__":
    main()
