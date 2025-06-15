# FloBot - Smart Menstrual Care System

## Quick Start

1. Install packages:
```bash
pip install paho-mqtt numpy pandas matplotlib scikit-learn
```

2. Start Node-RED:
```bash
npm install -g node-red
node-red
```

3. Run hardware controllers:
```bash
# On Raspberry Pi
python raspberry_pi/button_control.py
python raspberry_pi/moodAndpain.py
```

## Components

### Hardware
- Raspberry Pi + GrovePi+
- Grove buttons
- Grove LED strips
- Grove ultrasonic sensor

### Software
- Python for hardware control
- MQTT for communication
- Node-RED dashboard

## Features
- Period tracking (start/end buttons)
- Mood & pain level monitoring (10-level scale)
- Gesture recognition (No_Gesture/Hover)
- Real-time web dashboard

## Dashboard
Access at: `http://localhost:1880/ui`

## Note
Current implementation is for demonstration only:
- Node-RED nodes need correct server addresses
- Python MQTT server addresses need to be updated
- Default addresses are set to localhost for testing
