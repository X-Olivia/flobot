#!/usr/bin/env python3
"""
Hardware control module for menstrual care robot.
Controls two buttons for mood/pain input and two LED strips for visual feedback.
"""


import time
import json
import grovepi
import paho.mqtt.client as mqtt
from datetime import datetime
from rpi_ws281x import *
import board

# GPIO Pin configuration
MOOD_BUTTON = 4    # D4 port for mood button
PAIN_BUTTON = 5    # D5 port for pain button
MOOD_LED_PIN = 19 # GPIO pin for mood LED strip
PAIN_LED_PIN = 18  # GPIO pin for pain LED strip


# LED strip configuration
MAX_LEDS = 10      # Number of LEDs in each strip
LED_FREQ_HZ = 800000  # LED signal frequency in hertz
LED_DMA = 10         # DMA channel to use for generating signal
LED_BRIGHTNESS = 30  # Set to 0 for darkest and 255 for brightest
LED_INVERT = False   # True to invert the signal
LED_CHANNEL_1 = 1    # For mood LED strip
LED_CHANNEL_2 = 0    # For pain LED strip


# MQTT configuration
MQTT_BROKER = "10.148.187.242"  # change here
MQTT_PORT = 1883
MQTT_TOPIC_MOOD = "sensor/mood"  # Topic for publishing mood data
MQTT_TOPIC_PAIN = "sensor/pain"  # Topic for publishing pain data


# Mood levels (10 levels) - Aligned with Node-RED implementation
MOOD_LEVELS = [
   "Calm",       # LED 1
   "Happy",      # LED 2
   "Excited",    # LED 3
   "Anxious",    # LED 4
   "Sad",        # LED 5
   "Tired",      # LED 6
   "Irritated",  # LED 7
   "Angry",      # LED 8
   "Depressed",  # LED 9
   "Stressed"    # LED 10
]


class HardwareController:
   def __init__(self):
       # Initialize MQTT client with new API version
       self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
       self.client.on_connect = self.on_connect
       self.client.on_publish = self.on_publish
       self.client.on_message = self.on_message


       # Initialize counters for LED control
       self.mood_count = 0  # 0-10, 0 means no LEDs lit
       self.pain_count = 0  # 0-10, 0 means no LEDs lit


       # Initialize LED strips
       self.mood_strip = Adafruit_NeoPixel(MAX_LEDS, MOOD_LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL_1)
       self.pain_strip = Adafruit_NeoPixel(MAX_LEDS, PAIN_LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL_2)
       self.mood_strip.begin()
       self.pain_strip.begin()


       # Setup GPIO pins
       self.setup_gpio()


       # Connect to MQTT broker
       try:
           print(f"Connecting to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
           self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
           self.client.loop_start()
       except Exception as e:
           print(f"Failed to connect to MQTT broker: {e}")
           exit(1)


   def setup_gpio(self):
       """Setup GPIO pins for buttons and LED strips"""
       try:
           # Set button pins as INPUT
           grovepi.pinMode(MOOD_BUTTON, "INPUT")
           grovepi.pinMode(PAIN_BUTTON, "INPUT")
          
           # Initialize LED strips to OFF
           self.update_led_strip("mood", 0)
           self.update_led_strip("pain", 0)
          
       except Exception as e:
           print(f"Error setting up GPIO: {e}")
           exit(1)
  
   def update_led_strip(self, strip_type, count):
       """Update LED strip to show specified number of LEDs"""
       try:
           strip = self.mood_strip if strip_type == "mood" else self.pain_strip
          
           # Turn off all LEDs first
           for i in range(MAX_LEDS):
               strip.setPixelColor(i, Color(0, 0, 0))
          
           # Turn on LEDs up to count
           for i in range(count):
               if strip_type == "mood":
                   strip.setPixelColor(i, Color(0, 255, 0))  # Green for mood
               else:
                   strip.setPixelColor(i, Color(255, 0, 0))  # Red for pain
          
           strip.show()
          
       except Exception as e:
           print(f"Error updating LED strip: {e}")


   def on_connect(self, client, userdata, flags, rc, properties):
       """Callback when connected to MQTT broker"""
       if rc == 0:
           print("Successfully connected to MQTT broker")
           self.client.subscribe("sensor/status")  # Subscribe to status updates
       else:
           print(f"Failed to connect to MQTT broker with result code: {rc}")


   def on_publish(self, client, userdata, mid, reason_code, properties):
       """Callback when message is published"""
       print(f"Message {mid} published successfully")


   def on_message(self, client, userdata, msg):
       """handle received messages, synchronize LED status"""
       print("message is", msg.topic, msg.payload)
       try:
           if msg.topic == "sensor/status":
               data = json.loads(msg.payload)
               # update local state
               mood_led_index = next((i for i, led in enumerate(data['mood_leds']) if led == 1), -1)
               print("mood led index is", mood_led_index)
               pain_led_index = next((i for i, led in enumerate(data['pain_leds']) if led == 1), -1)
              
               # update counter and LED display
               self.mood_count = mood_led_index + 1 if mood_led_index >= 0 else 0
               self.pain_count = pain_led_index + 1 if pain_led_index >= 0 else 0
              
               # updateLED
               self.update_led_strip("mood", self.mood_count)
               print("led_pin ", self.mood_count, "arg 2", "mood")
               self.update_led_strip("pain", self.pain_count)
              
               print(f"Status synchronized - Mood: {self.mood_count} ({MOOD_LEVELS[self.mood_count - 1] if self.mood_count > 0 else 'None'})")
               print(f"                    Pain: {self.pain_count}")
       except Exception as e:
           print(f"Error processing message: {e}")
           print(f"Received message: {msg.payload}")


   def publish_mood_state(self):
       """Publish mood button state and LED status"""
       print("publishing mood state")
       try:
           # Create LED status array (only one LED on at current level)
           led_status = [0] * MAX_LEDS
           if self.mood_count > 0:
               led_status[self.mood_count - 1] = 1


           # Create message payload
           payload = {
               "timestamp": datetime.now().isoformat(),
               "level": self.mood_count,
               "mood": MOOD_LEVELS[self.mood_count - 1] if self.mood_count > 0 else "None",
               "leds": led_status
           }
          
           # Publish message
           result = self.client.publish(MQTT_TOPIC_MOOD, json.dumps(payload))
           if result.rc == mqtt.MQTT_ERR_SUCCESS:
               mood_status = MOOD_LEVELS[self.mood_count - 1] if self.mood_count > 0 else "None"
               print(f"\n{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} - Mood Level: {self.mood_count} ({mood_status})")
           else:
               print(f"Failed to publish message, error code: {result.rc}")
       except Exception as e:
           print(f"Error publishing message: {e}")


   def publish_pain_state(self):
       """Publish pain button state and LED status"""
       try:
           # Create LED status array (only one LED on at current level)
           led_status = [0] * MAX_LEDS
           if self.pain_count > 0:
               led_status[self.pain_count - 1] = 1


           # Create message payload
           payload = {
               "timestamp": datetime.now().isoformat(),
               "level": self.pain_count,
               "leds": led_status
           }
          
           # Publish message
           result = self.client.publish(MQTT_TOPIC_PAIN, json.dumps(payload))
           if result.rc == mqtt.MQTT_ERR_SUCCESS:
               print(f"\n{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} - Pain Level: {self.pain_count}")
           else:
               print(f"Failed to publish message, error code: {result.rc}")
       except Exception as e:
           print(f"Error publishing message: {e}")


   def run(self):
       """Main control loop"""
       print("\nHardware Controller Running...")
       print("Press Ctrl+C to exit")
       print("\nMood Levels:")
       for i, mood in enumerate(MOOD_LEVELS, 1):
           print(f"{i}. {mood}")


       last_mood_state = 0
       last_pain_state = 0


       try:
           while True:
               try:
                   # Read button states
                   mood_state = grovepi.digitalRead(MOOD_BUTTON)
                   pain_state = grovepi.digitalRead(PAIN_BUTTON)


                   # Handle mood button press
                   if mood_state and not last_mood_state:  # Button just pressed
                       self.mood_count = (self.mood_count + 1) % (MAX_LEDS + 1)
                       self.update_led_strip("mood", self.mood_count)
                       self.publish_mood_state()
                       time.sleep(0.2)  # Debounce delay


                   # Handle pain button press
                   if pain_state and not last_pain_state:  # Button just pressed
                       self.pain_count = (self.pain_count + 1) % (MAX_LEDS + 1)
                       self.update_led_strip("pain", self.pain_count)
                       self.publish_pain_state()
                       time.sleep(0.2)  # Debounce delay


                   # Update last states
                   last_mood_state = mood_state
                   last_pain_state = pain_state


                   # Small delay to prevent CPU overload
                   time.sleep(0.1)


               except Exception as e:
                   print(f"Error reading buttons: {e}")
                   time.sleep(1)


       except KeyboardInterrupt:
           print("\nExiting...")
       finally:
           self.cleanup()


   def cleanup(self):
       """Cleanup resources"""
       # Turn off LED strips
       try:
           self.update_led_strip("mood", 0)
           self.update_led_strip("pain", 0)
       except:
           pass


       # Disconnect MQTT
       self.client.loop_stop()
       self.client.disconnect()
       print("Disconnected from MQTT broker")


if __name__ == "__main__":
   controller = HardwareController()
   controller.run()









