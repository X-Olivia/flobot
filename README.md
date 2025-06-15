# FloBot: Menstrual Cycle Assistant Robot
Hi, I'm the developer of this project, my team mates made the lovely presentation video, you can watch it to know how it really look like. However, here's the introduction to help you understand the details well:

## Features

### Dashboard Interface

1. **Period Record Tracking**
   - Log the start and end dates of menstrual periods
   - Track cycle history over time
   - Automatic data persistence to local storage (~/.node-red/data/records.json)
   - Display last 10 historical records
   - Each record includes:
     - Date and timestamp
     - Status (start/end)
     - Pain level
     - Mood state

2. **Mood and Pain Monitoring**
   - **Mood Tracking**
     - 10-level LED indicator display
     - Supported mood states: Calm, Happy, Excited, Anxious, Sad, Tired, Irritated, Angry, Depressed, Stressed
     - Increment mood levels via "Mood +" button
     - Reset capability with "Reset Mood" button
   - **Pain Tracking**
     - 10-level LED indicator display
     - Pain scale from 0 to 10
     - Increment pain levels via "Pain +" button
     - Reset capability with "Reset Pain" button
   - Real-time visualization of both indicators
   - Data automatically syncs with period records

3. **AI Chatbot Assistant**
   - Ask questions about menstrual health
   - Get personalized advice and support
   - Access information about symptoms and cycle phases

4. **TurtleBot Remote Control**
- **Manual Control Mode**
   - Control via Node-RED dashboard directional buttons
   - Four-direction support:
     - Forward (↑)
     - Backward (↓)
     - Left turn (←)
     - Right turn (→)
   - Velocity Parameters:
     - Linear velocity (linear.x): ±0.2 m/s
     - Angular velocity (angular.z): ±0.5 rad/s
   - Auto-stop on button release
   - Real-time camera feed for visual feedback

- **Predefined Location Mode**
   - One-click navigation to preset positions
   - Current supported locations:
     - Position 1 (Bed): (0.520, 0.340, 0.932)
     - Position 2 (Bathroom): (2.140, 1.300, 0.000)
   - Autonomous navigation using ROS move_base

5. **Cycle Prediction and Analysis**
   - Predict entire menstrual cycle based on recorded data
   - Identify the four main phases:
     - Menstrual phase
     - Follicular phase
     - Ovulation phase
     - Luteal phase
   - Provide daily recommendations and encouragement based on current cycle phase

### Data Management Features
- Automatic data saving and loading
- Individual record deletion capability
- Automatic reset of mood and pain indicators upon record deletion
- Persistent storage with history recovery after restart
- Tabular display of historical records with timestamp, period status, pain level, and mood state
- Chronological sorting with newest records first
- Data-driven cycle predictions and personalized health recommendations
- Visual representation of cycle phases


## Communication Architecture

### MQTT Topics and Node Structure

1. **Robot Control Communication**
   - Published Topics:
     - `cmd_vel`: Robot movement control commands (linear and angular velocity)
     - `robot/command`: Predefined position movement commands
   - Subscribed Topics:
     - `robot/ack`: Robot position arrival confirmation
   - Related Nodes:
     - MQTT Out node "Robot Control Command"
     - MQTT Out node "Robot Command"
     - MQTT In node "Position Status"

2. **Sensor Data Communication**
   - Subscribed Topics:
     - `sensor/mood`: Mood sensor data
     - `sensor/pain`: Pain sensor data
     - `sensor/gesture`: Gesture recognition data
   - Related Nodes:
     - MQTT In node "Mood Button Input"
     - MQTT In node "Pain Button Input"
     - MQTT In node "Sensor Input"

3. **Camera Stream Communication**
   - Subscribed Topics:
     - `camera/rgb`: RealSense camera RGB image stream
   - Related Nodes:
     - MQTT In node "Camera Stream Input"
     - UI Template node for camera display

4. **LED Control Communication**
   - Published Topics:
     - `led/ring`: LED ring state control commands
   - Related Nodes:
     - MQTT Out node "LED Ring Control"

5. **Period Status Communication**
   - Subscribed Topics:
     - `period/status`: Period status changes (start/end)
   - Related Nodes:
     - MQTT In node "Button Input"
     - Function node "Handle Period Status"

### Data Processing Nodes

1. **Function Nodes**
   - "Process Sensor Data": Handles sensor data and UI updates
   - "Process Period Data": Manages period records and storage
   - "Calculate Prediction": Computes cycle predictions
   - "Process Robot Control": Handles robot control commands

2. **UI Nodes**
   - UI Template nodes for LED ring status display
   - UI Button nodes for interface interaction
   - UI Text nodes for status information display

3. **Data Persistence**
   - Function node "File Operations": Handles data file I/O
   - Function node "Initialize Records": Sets up historical records

### MQTT Broker Configuration
- Server Address: 10.148.187.227
- Port: 1883
- QoS Levels:
  - Sensor data and control commands: QoS 2
  - Video stream: QoS 0


## Technology Stack

- Node-RED for dashboard interface and control logic
- TurtleBot for physical assistance
- AI-powered chatbot for personalized support
- Local data persistence for reliable record keeping


## Hardware Interface

### Main Controller
- **Raspberry Pi**
  - Primary functions: LED control, button input, MQTT communication
  - Equipped with: GrovePi+ expansion board

### GrovePi+ Port Assignments
- **Button Connections**
  - Period start button: D2 port
  - Period end button: D3 port
  - Mood input button: D4 port
  - Pain input button: D5 port

### GPIO Port Assignments (Raspberry Pi)
- **LED Strip Connections**
  - Mood LED strip: GPIO 18 (PWM)
  - Pain LED strip: GPIO 19 (PWM)
  - Cycle prediction LED ring: GPIO 21 (PWM)
- **LED Configurations**
  - Mood/Pain indicators: 10 LEDs each
  - Cycle prediction ring: 20 LEDs (WS2813 Mini)

### TurtleBot Configuration
- **Base Platform**: TurtleBot 3 Waffle Pi
- **Camera**: Intel RealSense D435i
  - Mounting: Top front of TurtleBot
  - Functions: Real-time video streaming
- **Communication**
  - ROS network communication with Raspberry Pi
  - Shared MQTT broker

## Future Plan
- I will keep this project as my FYP:)
