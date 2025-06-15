#!/usr/bin/env python3
"""
Movement control node for TurtleBot.
Handles navigation to predefined points and gesture-based control.
"""

import rospy
import math
import actionlib
import json
import paho.mqtt.client as mqtt
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion
import time

# MQTT Configuration
MQTT_BROKER = "10.148.187.242"
MQTT_PORT = 1883
MQTT_TOPIC = "robot/command"  # Topic to receive commands
MQTT_ACK_TOPIC = "robot/ack"  # Topic for sending acknowledgments
MQTT_CMD_VEL_TOPIC = "cmd_vel"  # Topic for manual control

# Predefined points (x, y, orientation)
POINTS = {
    'position1': (0.5200000405311584, 0.33999988436698914, 0.9319747691158558),  # Position 1
    'position2': (2.140000343322754, 1.2999998331069946, 0.0),  # Position 2
}

class MovementController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('movement_controller', anonymous=True)
        rospy.loginfo("Initializing TurtleBot movement controller...")
        
        # Publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        
        # Action client for navigation
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        print("started move base client")

        
        #rospy.loginfo("Connected to move_base action server")
        
        # Initialize MQTT client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message

        #self.move_base_client.wait_for_server()
        
        # Connect to MQTT broker
        try:
            rospy.loginfo(f"Attempting to connect to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
            rospy.loginfo(f"Connected to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
        except Exception as e:
            rospy.logerr(f"Failed to connect to MQTT broker: {e}")
        
        # State variables
        self.current_pose = None
        self.current_position = None  # Track current named position
        self.is_connected = False
        self.is_manual_control = False  # Flag for manual control mode

    def send_ack(self, gesture, position, status):
        """Send acknowledgment message to Node-RED"""
        try:
            ack_message = {
                "gesture": gesture,
                "position": position,
                "status": status,
                "timestamp": time.strftime('%Y-%m-%d %H:%M:%S')
            }
            result = self.mqtt_client.publish(MQTT_ACK_TOPIC, json.dumps(ack_message))
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                rospy.loginfo(f"Sent acknowledgment: {ack_message}")
                # Update the controller's current position state
                if status == "completed" and position in POINTS:
                    self.current_position = position
                    rospy.loginfo(f"Position state updated to: {self.current_position}")
            else:
                rospy.logerr(f"Failed to send acknowledgment, error code: {result.rc}")
        except Exception as e:
            rospy.logerr(f"Error sending acknowledgment: {e}")

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback when connected to MQTT broker"""
        print("on_MQTT_connect")
        if rc == 0:
            rospy.loginfo("Successfully connected to MQTT broker")
            rospy.loginfo(f"Subscribing to topics: {MQTT_TOPIC}, {MQTT_CMD_VEL_TOPIC}")
            self.mqtt_client.subscribe([(MQTT_TOPIC, 0), (MQTT_CMD_VEL_TOPIC, 0)])
            self.is_connected = True
            self.send_ack("system", "none", "connected")
        else:MQTT_CMD_VEL_TOPICFalse

    def on_mqtt_message(self, client, userdata, msg):
        """Handle incoming MQTT messages"""
        try:
            rospy.loginfo(f"Received message on topic {msg.topic}: {msg.payload.decode()}")
            print("ONMQTT")
            
            # Handle manual control messages
            if msg.topic == MQTT_CMD_VEL_TOPIC:
                self.handle_manual_control(msg.payload.decode())
                return
            
            # Handle navigation commands
            if msg.topic == MQTT_TOPIC:
                self.is_manual_control = False
                data = json.loads(msg.payload.decode())
                gesture = data.get('gesture')
                position = data.get('position')
                
                if not gesture or not position:
                    rospy.logwarn("Missing gesture or position in message")
                    return
                    
                rospy.loginfo(f"Processing command - Gesture: {gesture}, Position: {position}")
                
                # Process "move" command 
                if gesture == 'move' and position in POINTS:
                    # If we're already at the requested position, don't move
                    if position == self.current_position:
                        rospy.loginfo(f"Already at position: {position}, no movement needed")
                        self.send_ack(gesture, position, "already_at_position")
                    else:
                        rospy.loginfo(f"Starting movement to position: {position}")
                        self.move_to_point(position)
                else:
                    rospy.logwarn(f"Unknown position or gesture: {position}, {gesture}")
                    self.send_ack(gesture, position, "error: unknown command")
            
        except json.JSONDecodeError as e:
            rospy.logerr(f"Failed to parse MQTT message: {e}")
            self.send_ack("unknown", "unknown", "error: invalid message format")
        except Exception as e:
            rospy.logerr(f"Error processing MQTT message: {e}")
            self.send_ack("unknown", "unknown", f"error: {str(e)}")

    def handle_manual_control(self, payload):
        """Process manual control commands"""
        try:
            cmd = json.loads(payload)
            print(cmd)
            print((cmd.get('linear', 0.0)))
            print((cmd.get('angular', 0.0)))
            
            # Create and publish Twist message
            twist = Twist()
            twist.linear.x = float(cmd.get('linear', 0.0))
            twist.angular.z = float(cmd.get('angular', 0.0))
            
            # Cancel any ongoing navigation if manual control is initiated
            if abs(twist.linear.x) > 0.0 or abs(twist.angular.z) > 0.0:
                if not self.is_manual_control:
                    self.is_manual_control = True
                    self.move_base_client.cancel_all_goals()
                    rospy.loginfo("Switching to manual control mode")
            else:
                self.is_manual_control = False
            
            # Publish velocity command
            self.cmd_vel_pub.publish(twist)          
            
        except Exception as e:
            rospy.logerr(f"Error processing manual control command: {e}")

    def odom_callback(self, msg):
        """Process odometry data"""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Convert quaternion to Euler angles
        _, _, yaw = euler_from_quaternion([
            orientation.x, orientation.y,
            orientation.z, orientation.w
        ])
        
        self.current_pose = (position.x, position.y, yaw)

    def move_to_point(self, point_name):
        """Navigate to predefined point"""
        if self.is_manual_control:
            rospy.logwarn("Cannot start navigation while in manual control mode")
            self.send_ack("movement", point_name, "error: manual control active")
            return False
            
        if point_name not in POINTS:
            rospy.logerr(f"Unknown point: {point_name}")
            self.send_ack("movement", point_name, "error: unknown position")
            return False
        
        x, y, orientation = POINTS[point_name]
        rospy.loginfo(f"Moving to {point_name}: ({x}, {y}, {orientation})")
        
        # Create goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # Set position
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        
        # Set orientation (convert to quaternion)
        goal.target_pose.pose.orientation.z = math.sin(orientation/2)
        goal.target_pose.pose.orientation.w = math.cos(orientation/2)
        
        # Send goal and wait
        self.move_base_client.send_goal(goal)
        rospy.loginfo(f"Goal sent to move_base for {point_name}")
        
        # Send status update
        self.send_ack("movement", point_name, "moving")
        
        success = self.move_base_client.wait_for_result()
        
        if success:
            rospy.loginfo(f"Successfully reached {point_name}")
            self.current_position = point_name  # Update current position
            self.send_ack("movement", point_name, "completed")
        else:
            rospy.logerr(f"Failed to reach {point_name}")
            self.send_ack("movement", point_name, "failed")
        
        return success

    def run(self):
        """Main loop"""
        try:
            rospy.loginfo("Movement controller is running")
            rospy.loginfo(f"Waiting for commands on topics: {MQTT_TOPIC}, {MQTT_CMD_VEL_TOPIC}")
            
            # Periodically check MQTT connection
            rate = rospy.Rate(1)  # 1 Hz
            while not rospy.is_shutdown():
                if not self.is_connected:
                    rospy.logwarn("MQTT connection lost, attempting to reconnect...")
                    try:
                        self.mqtt_client.reconnect()
                    except Exception as e:
                        rospy.logerr(f"Reconnection failed: {e}")
                rate.sleep()
                
        except rospy.ROSInterruptException:
            pass
        finally:
            rospy.loginfo("Shutting down...")
            self.send_ack("system", "none", "disconnected")
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            rospy.loginfo("Shutdown complete")

if __name__ == '__main__':
    try:
        controller = MovementController()
        controller.run()
    except Exception as e:
        rospy.logerr(f"Movement controller failed: {e}")













