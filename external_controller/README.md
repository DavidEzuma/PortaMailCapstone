# external_controller

Reference external controller implementation using the LCD JSON contract.
It polls the LCD server and drives mode changes with simple logic.

## How to run
1) Start the Flask server in `LCD_web/portamail_ui`.
2) Run:
```bash
python3 external_controller/controller.py
```

This uses HTTP polling and can later be replaced by the ROS2 bridge or a C++ client.
