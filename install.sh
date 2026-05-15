#!/bin/bash
set -e

echo "=== modbus-proxy with SunSpec conversion installer ==="

# Install dependencies
apt-get update -qq
apt-get install -y -qq python3 python3-pip python3-venv

# Create venv
python3 -m venv /opt/modbus-proxy
/opt/modbus-proxy/bin/pip install --quiet pyyaml

# Download proxy script
curl -fsSL https://raw.githubusercontent.com/ChristophCaina/modbus-proxy/format-changer/modbus_proxy.py \
    -o /opt/modbus-proxy/modbus_proxy.py

# Create default config if not exists
if [ ! -f /etc/modbus-proxy.yml ]; then
    cat > /etc/modbus-proxy.yml << 'YAML'
devices:
  - modbus:
      url: 192.168.52.41:1502
    listeners:
      # int16 — Home Assistant, OpenWB
      - bind: 0:5020
      # float32 — Bosch Energy Manager
      - bind: 0:5021
        register_conversions:
          - address: 40083     # I_AC_Power
            sf_address: 40084  # I_AC_Power_SF
            source_type: int16
            target_type: float32
          - address: 40206     # M_AC_Power (Meter)
            sf_address: 40210  # M_AC_Power_SF
            source_type: int16
            target_type: float32
YAML
    echo "Created default config at /etc/modbus-proxy.yml — please adjust the modbus URL!"
else
    echo "Config /etc/modbus-proxy.yml already exists, skipping."
fi

# Systemd service
cat > /etc/systemd/system/modbus-proxy.service << 'SERVICE'
[Unit]
Description=ModBus Proxy with SunSpec Conversion
After=network.target

[Service]
Type=simple
Restart=always
ExecStart=/opt/modbus-proxy/bin/python3 /opt/modbus-proxy/modbus_proxy.py -c /etc/modbus-proxy.yml

[Install]
WantedBy=multi-user.target
SERVICE

systemctl daemon-reload
systemctl enable modbus-proxy
systemctl restart modbus-proxy

echo ""
echo "=== Done! ==="
echo "Config:  /etc/modbus-proxy.yml"
echo "Status:  systemctl status modbus-proxy"
echo "Logs:    journalctl -u modbus-proxy -f"
