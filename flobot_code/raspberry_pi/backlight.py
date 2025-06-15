#!/usr/bin/env python3

import time
import json
import paho.mqtt.client as mqtt
import smbus
import sys

# I2C setup
bus = smbus.SMBus(1)  # Use I2C bus 1

DISPLAY_TEXT_ADDR = 0x3e  # LCD text address
DISPLAY_RGB_ADDR = 0x30   # RGB backlight address

# Cycle phases and their colors
CYCLE_PHASES = {
    "Menstrual": [255, 100, 100],   # Soft red
    "Follicular": [150, 255, 150],  # Soft green
    "Ovulation": [255, 255, 100],   # Soft yellow
    "Luteal": [100, 150, 255]       # Soft blue
}

# Global variables to store phase information
current_phase = "Unknown"

def initialize_lcd():
    """Initialize LCD"""
    try:
        # Clear display
        bus.write_byte_data(DISPLAY_TEXT_ADDR, 0x80, 0x01)
        time.sleep(0.1)
        
        # Display on, no cursor
        bus.write_byte_data(DISPLAY_TEXT_ADDR, 0x80, 0x0C)
        time.sleep(0.1)
        
        # Entry mode: move cursor right
        bus.write_byte_data(DISPLAY_TEXT_ADDR, 0x80, 0x06)
        time.sleep(0.1)
        
        # Function set: 2 lines, 5x8 dots
        bus.write_byte_data(DISPLAY_TEXT_ADDR, 0x80, 0x28)
        time.sleep(0.1)
        
        print("LCD initialization complete")
    except Exception as e:
        print(f"LCD initialization error: {e}")
        return False
    return True

def set_rgb(r, g, b):
    """Set RGB backlight color"""
    try:
        # Initialize RGB controller
        bus.write_byte_data(DISPLAY_RGB_ADDR, 0, 0)
        bus.write_byte_data(DISPLAY_RGB_ADDR, 1, 0)
        
        # Set all LEDs to work
        bus.write_byte_data(DISPLAY_RGB_ADDR, 0x08, 0xAA)
        
        # Set RGB values
        bus.write_byte_data(DISPLAY_RGB_ADDR, 4, r)  # Red
        bus.write_byte_data(DISPLAY_RGB_ADDR, 3, g)  # Green
        bus.write_byte_data(DISPLAY_RGB_ADDR, 2, b)  # Blue
        
        return True
    except Exception as e:
        print(f"Error setting RGB: {e}")
        return False

def set_text(text):
    """Display text on the LCD"""
    try:
        # Clear display
        bus.write_byte_data(DISPLAY_TEXT_ADDR, 0x80, 0x01)
        time.sleep(0.1)
        
        # Set text centered on the display
        bus.write_byte_data(DISPLAY_TEXT_ADDR, 0x80, 0x80)  # Set DDRAM address to first line start
        for char in text[:16]:  # Limit to 16 characters
            bus.write_byte_data(DISPLAY_TEXT_ADDR, 0x40, ord(char))
        
        return True
    except Exception as e:
        print(f"Error setting text: {e}")
        return False

def update_display():
    """Update display based on current phase"""
    global current_phase
    
    if current_phase == "Unknown":
        set_text("Waiting for data")
        set_rgb(100, 100, 100)  # Gray for no data
        return
    
    # Get phase color
    color = CYCLE_PHASES.get(current_phase, [100, 100, 100])
    set_rgb(color[0], color[1], color[2])
    
    # Display only the phase name
    set_text(current_phase)

def get_phase_name(day):
    """Get phase name based on cycle day"""
    if day <= 6:
        return "Menstrual"
    elif day <= 13:
        return "Follicular"
    elif day <= 16:
        return "Ovulation"
    else:
        return "Luteal"

def on_message(client, userdata, msg):
    """Process incoming MQTT messages"""
    global current_phase
    
    topic = msg.topic
    
    try:
        # Handle period status updates
        if topic == "period/status":
            status = msg.payload.decode()
            print(f"Period status update: {status}")
            
            if status == "start":
                current_phase = "Menstrual"
            elif status == "end":
                current_phase = "Follicular"
            
            # Update display
            update_display()
            return
        
        # Handle LED ring data
        if topic == "led/ring":
            payload = json.loads(msg.payload.decode())
            has_prediction = payload.get("has_prediction", False)
            
            if has_prediction:
                current_day = payload.get("current_cycle_day", 0)
                current_phase = get_phase_name(current_day)
                
                print(f"Updated cycle info: Day {current_day}, Phase: {current_phase}")
                
                # Update display
                update_display()
    
    except Exception as e:
        print(f"Error processing message: {e}")

def on_connect(client, userdata, flags, rc, properties=None):
    """Callback when connected to MQTT broker"""
    if rc == 0:
        print("Successfully connected to MQTT broker")
        # Subscribe to topics
        client.subscribe("period/status", qos=0)
        client.subscribe("led/ring", qos=0)
        # Request initial data
        client.publish("request/refresh", "1")
    else:
        print(f"Failed to connect to MQTT broker with result code: {rc}")

def main():
    """Main function"""
    print("Starting Grove LCD backlight program...")
    
    # Initialize LCD
    if not initialize_lcd():
        print("LCD initialization failed, exiting program")
        return
    
    # Show welcome message
    set_rgb(50, 50, 50)
    set_text("Connecting to MQTT")
    
    # Connect to MQTT broker
    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        client.on_message = on_message
        client.on_connect = on_connect
        
        # Connect to MQTT broker
        client.connect("10.148.187.242", 1883, 60)
        client.loop_start()
    except Exception as e:
        print(f"Error connecting to MQTT broker: {e}")
        set_text("MQTT connect failed")
        set_rgb(255, 0, 0)  # Red for error
        time.sleep(5)
        return
    
    try:
        # Main loop - doesn't do much, everything handled by callbacks
        while True:
            time.sleep(60)  # Check once per minute
            
    except KeyboardInterrupt:
        print("Program terminated by user")
    except Exception as e:
        print(f"Main loop error: {e}")
    finally:
        # Clean up
        if 'client' in locals():
            client.loop_stop()
            client.disconnect()
        
        # Clear display
        set_text("")
        set_rgb(0, 0, 0)
        print("Program ended")

if __name__ == "__main__":
    main()
