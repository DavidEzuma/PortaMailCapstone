#!/bin/bash
set -e

# Update splashscreen service to use the Python animated splash
cat > /etc/systemd/system/splashscreen.service << 'EOF'
[Unit]
Description=PortaMail boot splash screen
DefaultDependencies=no
After=local-fs.target

[Service]
ExecStart=/usr/bin/python3 /home/davidezuma/PortaMailCapstone/scripts/splashscreen.py
StandardInput=tty
StandardOutput=tty
StandardError=journal

[Install]
WantedBy=sysinit.target
EOF

systemctl daemon-reload
systemctl enable splashscreen.service
echo "Splash service updated."

# Clean up temp files
rm -f /home/davidezuma/fix_cmdline.py
rm -f /home/davidezuma/setup_silent_boot.sh
rm -f /home/davidezuma/apply_splash.sh
echo "Temp files removed."
