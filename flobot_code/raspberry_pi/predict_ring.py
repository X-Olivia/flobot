#!/usr/bin/env python3
"""
Hardware control module for Grove - RGB LED Ring (20 - WS2813 Mini).
Controls a 20-LED RGB ring for menstrual cycle phase visualization.
"""

import time
import json
import paho.mqtt.client as mqtt
from datetime import datetime
from rpi_ws281x import *

# LED ring configuration
LED_COUNT = 24        # Number of LED pixels
LED_PIN = 18         # GPIO pin connected to the pixels (must support PWM!)
LED_FREQ_HZ = 800000 # LED signal frequency in hertz
LED_DMA = 10         # DMA channel to use for generating signal
LED_BRIGHTNESS = 30 # Set to 0 for darkest and 255 for brightest
LED_INVERT = False   # True to invert the signal
LED_CHANNEL = 0      # PWM channel

# MQTT configuration
MQTT_BROKER = "10.148.187.242"  
MQTT_PORT = 1883
MQTT_TOPIC = "led/ring"

class LEDRingController:
    def __init__(self):
        # Initialize MQTT client
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        # Initialize LED states
        self.led_states = [(0, 0, 0)] * LED_COUNT  # RGB values for each LED

        # Initialize the LED strip
        self.strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
        self.strip.begin()

        # Connect to MQTT broker
        try:
            print(f"Connecting to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
            self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.client.loop_start()
        except Exception as e:
            print(f"Failed to connect to MQTT broker: {e}")
            exit(1)

    def on_connect(self, client, userdata, flags, rc, properties=None):
        """Callback when connected to MQTT broker"""
        if rc == 0:
            print("Successfully connected to MQTT broker")
            self.client.subscribe(MQTT_TOPIC)
            print(f"Subscribed to topic: {MQTT_TOPIC}")
        else:
            print(f"Failed to connect to MQTT broker with result code: {rc}")

    def on_message(self, client, userdata, msg):
        """Handle received MQTT messages"""
        try:
            data = json.loads(msg.payload)
            if "leds" in data:
                if not data.get("has_prediction", True):
                    # If no prediction, turn off all LEDs
                    self.update_leds([(0, 0, 0)] * LED_COUNT)
                    print("No prediction data available")
                    return

                # Convert hex colors to RGB values
                rgb_values = [self.hex_to_rgb(color) for color in data["leds"]]
                self.update_leds(rgb_values)
                self.print_status(data)
        except json.JSONDecodeError:
            print("Invalid JSON message received")
        except Exception as e:
            print(f"Error processing message: {e}")

    def hex_to_rgb(self, hex_color):
        """Convert hex color string to RGB tuple"""
        hex_color = hex_color.lstrip('#')
        return tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))

    def update_leds(self, rgb_values):
        """Update LED ring with new RGB values"""
        try:
            # Store new LED states
            self.led_states = rgb_values

            # Update each LED in the strip
            for i, (r, g, b) in enumerate(rgb_values):
                # Create 24-bit color value
                color = Color(r, g, b)
                self.strip.setPixelColor(i, color)
            
            # Show the updated colors
            self.strip.show()

        except Exception as e:
            print(f"Error updating LEDs: {e}")

    def print_status(self, data):
        """Print current LED ring status"""
        print(f"\nTime: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("LED Ring Status:")
        print("-" * 50)
        
        # Print cycle information if available
        if "current_cycle_day" in data:
            print(f"Current Cycle Day: {data['current_cycle_day']}")
            print(f"In Period: {'Yes' if data.get('in_period', False) else 'No'}")
        
        # Print color legend
        print("\nColor Legend:")
        print("● #f47c7c - Menstrual Period")
        print("● #a1de93 - Follicular Phase")
        print("● #f7f48b - Ovulation Phase")
        print("● #70a1d7 - Luteal Phase")
        print("-" * 50)

        # Print current LED colors
        for i, color in enumerate(data["leds"]):
            print(f"LED {i:2d}: {color}")

    def run(self):
        """Main control loop"""
        print("\nLED Ring Controller Running...")
        print("Press Ctrl+C to exit")
        
        try:
            while True:
                time.sleep(0.1)  # Prevent CPU overload
        except KeyboardInterrupt:
            print("\nExiting...")
        finally:
            self.cleanup()

    def cleanup(self):
        """Cleanup resources"""
        try:
            # Turn off all LEDs
            for i in range(LED_COUNT):
                self.strip.setPixelColor(i, Color(0, 0, 0))
            self.strip.show()
        except:
            pass

        # Disconnect MQTT
        self.client.loop_stop()
        self.client.disconnect()
        print("Disconnected from MQTT broker")

if __name__ == "__main__":
    controller = LEDRingController()
    controller.run()

