test folder for LCD branch

# PortaMail LCD HMI – Phase 1
This module implements the Phase 1 Human-Machine Interface (HMI) for the PortaMail robot using a Raspberry Pi–hosted web UI.

The LCD displays Room 1, Room 2, and Power buttons and reports user interactions as real-time binary states and edge events over Socket.IO.  
Each button press sets a corresponding bit while held and generates a timestamped event on release.

