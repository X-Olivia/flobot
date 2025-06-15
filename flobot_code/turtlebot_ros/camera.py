#!/usr/bin/env python3
"""
RealSense camera stream to Node-RED dashboard.
Captures RGB images and sends them via MQTT.
"""

import rospy
import cv2
import base64
import json
import paho.mqtt.client as mqtt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# MQTT Configuration
MQTT_BROKER = "10.148.187.242"
MQTT_PORT = 1883
MQTT_TOPIC = "camera/rgb"

class CameraStreamer:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('camera_streamer', anonymous=True)
        
        # Initialize MQTT client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_publish = self.on_mqtt_publish
        
        # Connect to MQTT broker
        try:
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
            rospy.loginfo(f"Connected to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
        except Exception as e:
            rospy.logerr(f"Failed to connect to MQTT broker: {e}")
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Subscribe to camera topic
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        rospy.loginfo("Subscribed to /camera/color/image_raw topic")
        
        # State variables
        self.frame_count = 0
        self.publish_rate = 5  # Publish every 5 frames

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback when connected to MQTT broker"""
        rospy.loginfo(f"Connected to MQTT broker with result code: {rc}")
        self.mqtt_client.subscribe(MQTT_TOPIC)
        rospy.loginfo(f"Subscribed to topic: {MQTT_TOPIC}")

    def on_mqtt_publish(self, client, userdata, mid):
        """Callback when message is published"""
        rospy.loginfo(f"Message published with mid: {mid}")

    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Only process every nth frame
            self.frame_count += 1
            if self.frame_count % self.publish_rate != 0:
                return
            
            # Resize image to reduce bandwidth
            height, width = cv_image.shape[:2]
            max_size = 640
            if width > max_size or height > max_size:
                scale = max_size / max(width, height)
                cv_image = cv2.resize(cv_image, (int(width * scale), int(height * scale)))
            
            # Convert image to base64
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            
            # Create message payload
            message = {
                "image": jpg_as_text,
                "timestamp": rospy.Time.now().to_sec(),
                "width": cv_image.shape[1],
                "height": cv_image.shape[0]
            }
            
            # Publish to MQTT
            result = self.mqtt_client.publish(MQTT_TOPIC, json.dumps(message))
            rospy.loginfo(f"Published image frame {self.frame_count} (mid: {result.mid})")
            
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def run(self):
        """Main loop"""
        try:
            rospy.loginfo("Camera streamer is running...")
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
        finally:
            # Cleanup
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()

if __name__ == '__main__':
    try:
        streamer = CameraStreamer()
        streamer.run()
    except Exception as e:
        rospy.logerr(f"Camera streamer failed: {e}") 