# FloBot Project Sensor Description

## Buttons

### 1. Period Start Button
- **Connection Port**: D2 (GrovePi+)
- **Function**: Records the start date of menstrual period
- **Implementation**: Monitors button state through `button_control.py`, sends "start" message to MQTT topic "period/status" when pressed
- **Selection Rationale**: 
  - Provides a simple and direct physical interaction method, allowing users to easily record the start of their period
  - Addresses a pain point identified in our questionnaire: most period tracking apps on the market require the complex process of "unlock phone-find app-open recording page-select-record", making it difficult for users to maintain consistent usage
  - One-button operation eliminates the need to unlock phones and navigate apps, significantly lowering the recording barrier

### 2. Period End Button
- **Connection Port**: D3 (GrovePi+)
- **Function**: Records the end date of menstrual period
- **Implementation**: Monitors button state through `button_control.py`, sends "end" message to MQTT topic "period/status" when pressed
- **Selection Rationale**: 
  - Pairs with the start button to form a complete period recording system, simplifying user operations
  - Solves the same recording complexity issues users face at the end of their period
  - Physical buttons provide tactile feedback, making them easier to operate without looking at the device compared to touchscreens

### 3. Mood Input Button
- **Connection Port**: D4 (GrovePi+)
- **Function**: Increments mood level counter, cycling through 10 mood states
- **Implementation**: Monitors button state through `moodAndpain.py`, increments mood level and updates LED display with each press
- **Selection Rationale**: 
  - Allows users to record complex emotional states through simple button operations, without requiring complicated input devices
  - Solves the problem of traditional apps requiring multiple clicks and selections for mood recording
  - Button and LED strip combination provides immediate visual feedback, enhancing the user recording experience

### 4. Pain Input Button
- **Connection Port**: D5 (GrovePi+)
- **Function**: Increments pain level counter, range 0-10
- **Implementation**: Monitors button state through `moodAndpain.py`, increments pain level and updates LED display with each press
- **Selection Rationale**: 
  - Provides an intuitive way to record pain levels, with physical buttons being more suitable for daily use scenarios
  - Addresses the issue that users may lack the energy to operate complex apps when experiencing period pain
  - Simplifies the pain recording process, encouraging users to record symptoms more frequently and accurately

## LED Strips

### 1. Mood LED Strip
- **Connection Port**: GPIO 18 (PWM)
- **Number of LEDs**: 10
- **Function**: Visually displays current mood state level
- **Implementation**: Controlled using the `rpi_ws281x` library, displays green LEDs to indicate mood level
- **Selection Rationale**: 
  - Provides intuitive visual feedback
  - 10 LEDs correspond to 10 mood states (Calm, Happy, Excited, Anxious, Sad, Tired, Irritated, Angry, Depressed, Stressed)
  - Green represents mood, aligning with psychological concepts of green representing balance and harmony
  - Solves the problem of unclear mood recording status in traditional apps, allowing users to see their current recorded mood state at a glance

### 2. Pain LED Strip
- **Connection Port**: GPIO 19 (PWM)
- **Number of LEDs**: 10
- **Function**: Visually displays current pain level (0-10)
- **Implementation**: Controlled using the `rpi_ws281x` library, displays red LEDs to indicate pain level
- **Selection Rationale**: 
  - Provides intuitive visual feedback
  - 10 LEDs correspond to pain levels 0-10
  - Red represents pain, aligning with intuitive understanding
  - Addresses the difficulty users have in recalling and accurately describing pain levels, helping users assess pain more objectively through visual scaling

## LED Ring

### Cycle Prediction LED Ring
- **Connection Port**: GPIO 21 (PWM)
- **Model**: Grove - RGB LED Ring (20 - WS2813 Mini)
- **Number of LEDs**: 22
- **Function**: Visually displays menstrual cycle prediction and current cycle phase
- **Implementation**: Controlled through `predict_ring.py`, displays different colors based on cycle prediction data calculated by Node-RED
- **Color Coding**:
  - Red (#f47c7c): Menstrual phase
  - Green (#a1de93): Follicular phase
  - Yellow (#f7f48b): Ovulation phase
  - Blue (#70a1d7): Luteal phase
- **Selection Rationale**: 
  - Ring design symbolizes cyclical nature
  - 22 LEDs can display cycle predictions for the next 22 days
  - Multi-color LEDs intuitively distinguish different cycle phases
  - Provides users with an intuitive understanding of their entire cycle
  - Solves the problem of traditional apps requiring opening and navigating to prediction pages; users only need to glance at the device to understand their cycle status

## LCD Display

### Grove LCD RGB Backlight Display
- **Connection**: I2C bus
  - Display text address: 0x3e
  - RGB backlight address: 0x30
- **Function**: Displays the current menstrual cycle phase name and enhances visual feedback through backlight color
- **Implementation**: Controlled through `backlight.py`, updates display content and backlight color based on cycle phase
- **Backlight Color Coding**:
  - Red: Menstrual phase
  - Green: Follicular phase
  - Yellow: Ovulation phase
  - Blue: Luteal phase
  - Gray: No data or waiting status
- **Selection Rationale**: 
  - Provides textual information to supplement LED visual feedback
  - Backlight color consistent with LED ring color coding, enhancing user understanding
  - I2C interface simplifies connections, reducing GPIO port usage
  - Combines dual feedback of text and color, suitable for different user preferences
  - Addresses users' vague understanding of cycle phases, enhancing comprehension of their physiological cycles through clear textual explanations

## Overall Sensor Selection Considerations

1. **User-Friendliness**:
   - Physical buttons are simple and intuitive to operate
   - LED displays provide immediate visual feedback
   - Display screen provides textual explanations, suitable for different user needs
   - Addresses app usage complexity issues identified in our questionnaire, greatly simplifying the recording process

2. **Information Hierarchy**:
   - Buttons: Input layer
   - LED strips: Immediate status feedback layer
   - LED ring: Prediction and cycle overview layer
   - Display screen: Textual explanation layer
   - Multi-level information presentation solves the problem of users needing to navigate multiple times in apps to access different information

3. **Technical Compatibility**:
   - All sensors are compatible with Raspberry Pi and GrovePi+
   - Uses standard interfaces (GPIO, PWM, I2C) for easy connection and maintenance
   - MQTT communication architecture ensures data synchronization between components

4. **Expandability**:
   - Modular design allows for future addition of more sensors
   - Software architecture supports new feature integration

5. **Reliability**:
   - Physical buttons are more reliable than touchscreens, suitable for daily use
   - LED displays are simple and intuitive, less prone to errors
   - Solves the problem of phone apps potentially being unusable due to system updates, battery depletion, etc.

6. **Addressing Real-Life Problems**:
   - According to our questionnaire, most period tracking apps on the market have the pain point of complex usage processes
   - Traditional apps require users to complete a series of steps: "unlock phone-find app-open recording page-select-record"
   - Complex recording processes cause many users to abandon consistent usage after a period of time
   - FloBot's physical hardware interface simplifies this process to "press a button," greatly lowering the recording barrier
   - Physical devices can be placed in private spaces such as bedrooms or bathrooms, making them more readily available when needed compared to phones
   - Dedicated devices avoid privacy concerns that might come with installing period apps on phones
