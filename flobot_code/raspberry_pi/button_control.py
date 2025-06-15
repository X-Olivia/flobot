#!/usr/bin/env python3
"""
Button control module for menstrual care robot.
Handles two buttons for period start/end recording.
"""

import time
import grovepi
import paho.mqtt.client as mqtt
from datetime import datetime

# GPIO Pin configuration
BUTTON_START = 2  # D2 port for period start button
BUTTON_END = 3    # D3 port for period end button

# MQTT configuration
MQTT_BROKER = "10.148.187.242"  
MQTT_PORT = 1883
MQTT_TOPIC = "period/status"  # Unified topic

class ButtonController:
    def __init__(self):
        # Initialize MQTT client with new API version
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect
        self.client.on_publish = self.on_publish

        # Connect to MQTT broker
        try:
            print(f"Connecting to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
            self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.client.loop_start()
        except Exception as e:
            print(f"Failed to connect to MQTT broker: {e}")
            exit(1)

        # Setup GPIO pins
        grovepi.pinMode(BUTTON_START, "INPUT")
        grovepi.pinMode(BUTTON_END, "INPUT")

    def on_connect(self, client, userdata, flags, rc, properties):
        """Callback when connected to MQTT broker"""
        if rc == 0:
            print("Successfully connected to MQTT broker")
        else:
            print(f"Failed to connect to MQTT broker with result code: {rc}")

    def on_publish(self, client, userdata, mid, reason_code, properties):
        """Callback when message is published"""
        print(f"Message {mid} published successfully")

    def run(self):
        """Main loop for button monitoring"""
        try:
            while True:
                # Check start button
                if grovepi.digitalRead(BUTTON_START):
                    self.client.publish(MQTT_TOPIC, "start")
                    print(f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} - Period start recorded")
                    time.sleep(1)  # Debounce

                # Check end button
                if grovepi.digitalRead(BUTTON_END):
                    self.client.publish(MQTT_TOPIC, "end")
                    print(f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} - Period end recorded")
                    time.sleep(1)  # Debounce

                time.sleep(0.1)  # Small delay to prevent CPU overload

        except KeyboardInterrupt:
            print("\nExiting...")
            self.cleanup()
        except Exception as e:
            print(f"Error: {e}")
            self.cleanup()

    def cleanup(self):
        """Cleanup resources"""
        self.client.loop_stop()
        self.client.disconnect()
        print("Disconnected from MQTT broker")

if __name__ == "__main__":
    controller = ButtonController()
    controller.run() 
