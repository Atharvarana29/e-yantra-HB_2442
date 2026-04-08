#!/usr/bin/env python3
'''
* Team Id: HB_2442
* Author List: Kumar Sushant Raj, Atharva Rana , Aditya Kumar
* Filename: mqtt_bridge_node.py
* Theme: eYRC Holo Battalion
* Functions: __init__,on_mqtt_connect(),on_mqtt_disconnect(),cmd_callback(),attach_callback(),on_message(),main()
* Global Variables: NONE
'''
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from hb_interfaces.msg import BotCmdArray, BotCmd
import paho.mqtt.client as mqtt
import json
from std_msgs.msg import Bool

# Ensure the IP matches with the MQTT Broker (Laptop IP)
BROKER_IP = "10.80.183.168"

class MqttBridge(Node):
    
    """
    ------------------------------------------------------------
    Function: __init__

    Input:
        None

    Output:
        None

    Logic Explanation:

        1. Initialize ROS2 node named 'mqtt_bridge_node'.

        2. Declare and retrieve robot_id parameter to allow
           per-robot configuration via launch files.

        3. Dynamically construct MQTT publish and subscribe
           topics using robot_id to isolate communication
           between multiple robots.

        4. Create ROS2 interfaces:
           - Subscribe to '/bot_cmd' for motion commands.
           - Provide attach service for gripper control.
           - Publish IR sensor status to controller.

        5. Initialize MQTT client and bind connection,
           disconnection, and message callbacks.

        6. Establish connection to MQTT broker and
           start asynchronous network loop.

        7. Subscribe to robot-specific sensor topic
           for real-time ESP feedback.

        8. Ensures bidirectional communication bridge
           between ROS2 control layer and ESP hardware.

    Example:
        MQTTBridgeNode()
    """
    def __init__(self):
        super().__init__('mqtt_bridge_node')
        self.get_logger().info("Starting MQTT Bridge Node")

        # ---------------- PARAMETERS ----------------
        # Default robot_id is 0. We are explicitly setting the robot_id in the launch files
        # for each robot (e.g., 0 for Crystal, 2 for Frostbite, 4 for Glacio)
        self.declare_parameter('robot_id', 0)
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().integer_value

        # ---------------- DYNAMIC TOPICS ----------------
        # Construct MQTT topics based on Robot ID
        # Example: if robot_id=0, topics are "esp/cmd/0" and "esp/sensor/0"
        self.cmd_topic_mqtt = f"esp/cmd/{self.robot_id}"
        self.sensor_topic_mqtt = f"esp/sensor/{self.robot_id}"

        self.get_logger().info(f"Bridge configured for Robot ID: {self.robot_id}")
        self.get_logger().info(f"MQTT Pub Topic: {self.cmd_topic_mqtt}")
        self.get_logger().info(f"MQTT Sub Topic: {self.sensor_topic_mqtt}")

        # ---------------- ROS 2 INTERFACE ----------------
        self.cmd_sub = self.create_subscription(
            BotCmdArray,
            '/bot_cmd',
            self.cmd_callback,
            10
        )

        self.attach_srv = self.create_service(
            SetBool,
            f'/robot_{self.robot_id}/attach',
            self.attach_callback
        )
        self.get_logger().info(f"🧲 Attach service ready at /robot_{self.robot_id}/attach")

        self.ir_pub = self.create_publisher(Bool, f'/robot_{self.robot_id}/ir_sensor_status', 10)

        # ---------------- MQTT SETUP ----------------
        self.solenoid_state = 0
        self.mqtt = mqtt.Client()
        self.mqtt.on_connect = self.on_mqtt_connect
        self.mqtt.on_disconnect = self.on_mqtt_disconnect
        self.mqtt.on_message = self.on_message

        try:
            self.mqtt.connect(BROKER_IP, 1883, 60)
            self.mqtt.loop_start()
            # Subscribe immediately to the specific sensor topic
            self.mqtt.subscribe(self.sensor_topic_mqtt)
        except Exception as e:
            self.get_logger().error(f"MQTT Connection failed: {e}")

    """
    ------------------------------------------------------------
    Function: on_mqtt_connect

    Input:
        client :
            MQTT client instance.
        userdata :
            Private user data (unused).
        flags :
            Response flags from broker.
        rc (int) :
            Connection result code.

    Output:
        None

    Logic Explanation:

        1. Check connection result code (rc).

        2. If rc == 0:
           - Confirm successful connection to broker.
           - Re-subscribe to robot-specific sensor topic
             to ensure continuity after reconnect.

        3. If connection fails:
           - Log error with return code for debugging.

        4. Maintains robust MQTT communication by
           handling reconnection automatically.

    Example:
        Automatically triggered by MQTT client on connect.
    """
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info(f"Connected to MQTT broker at {BROKER_IP}")
            # Re-subscribe on reconnect to ensure we don't lose the subscription
            client.subscribe(self.sensor_topic_mqtt)
        else:
            self.get_logger().error(f"MQTT connect failed rc={rc}")

    def on_mqtt_disconnect(self, client, userdata, rc):
        self.get_logger().warn("MQTT disconnected")
    
    """
    ------------------------------------------------------------
    Function: cmd_callback

    Input:
        msg (BotCmdArray) :
            Array of motor commands for multiple robots.

    Output:
        None

    Logic Explanation:

        1. Iterate through incoming command array
           to locate the command matching this robot_id.

        2. If no matching command exists, ignore
           the message safely.

        3. Convert the selected BotCmd message into
           JSON-compatible format.

        4. Publish serialized payload to the
           robot-specific MQTT command topic.

        5. Catch and log exceptions to prevent
           bridge crashes due to malformed messages.

        6. Ensures that each robot only receives
           its intended motor commands.

    Example:
        Triggered by '/bot_cmd' topic.
    """
    def cmd_callback(self, msg: BotCmdArray):
        """
        Processes /bot_cmd messages.
        Filters commands to find the one matching self.robot_id.
        """
        try:
            target_cmd = None

            # Find the command for THIS robot ID
            for cmd in msg.cmds:
                if cmd.id == self.robot_id:
                    target_cmd = cmd
                    break

            # If no command for this robot, ignore
            if target_cmd is None:
                return

            payload = self.botcmd_to_json(target_cmd)

            # Publish to specific MQTT topic
            self.mqtt.publish(self.cmd_topic_mqtt, json.dumps(payload), qos=0)

           

        except Exception as e:
            self.get_logger().error(f"cmd_callback error: {e}")
    
    """
    ------------------------------------------------------------
    Function: attach_callback

    Input:
        request (SetBool.Request) :
            Boolean request to enable or disable gripper.
        response (SetBool.Response) :
            Service response object.

    Output:
        SetBool.Response :
            Updated response with success status.

    Logic Explanation:

        1. Read requested boolean value from service call.

        2. Convert boolean into solenoid state:
           - True  → 1 (Activate gripper)
           - False → 0 (Deactivate gripper)

        3. Store updated solenoid state for use
           in outgoing MQTT command payload.

        4. Log updated state for debugging.

        5. Set response.success to True and
           provide confirmation message.

        6. Enables ROS2 service-based control
           of electromagnetic gripper.

    Example:
        Triggered by service '/robot_X/attach'.
    """
    def attach_callback(self, request, response):
        """Service callback for /attach (Gripper)"""
        self.solenoid_state = 1 if request.data else 0
        self.get_logger().info(f"[{self.robot_id}] Solenoid set to {self.solenoid_state}")

        response.success = True
        response.message = f"Solenoid state updated to {self.solenoid_state}"
        return response

    """
    ------------------------------------------------------------
    Function: botcmd_to_json

    Input:
        cmd (BotCmd) :
            ROS BotCmd message containing motor
            and servo commands.

    Output:
        dict :
            JSON-compatible dictionary payload
            for MQTT transmission.

    Logic Explanation:

        1. Extract wheel motor velocities (m1, m2, m3)
           and convert to float for JSON safety.

        2. Extract servo angles (base, elbow)
           and convert to integer format.

        3. Include current solenoid_state
           to control gripper activation.

        4. Return structured dictionary ready
           for serialization and MQTT publish.

        5. Ensures consistent data format between
           ROS2 controller and ESP firmware.

    Example:
        payload = self.botcmd_to_json(cmd)
    """
    def botcmd_to_json(self, cmd: BotCmd):
        """Helper to convert ROS BotCmd to JSON dict"""
        return {
            "m1": float(cmd.m1),
            "m2": float(cmd.m2),
            "m3": float(cmd.m3),
            "base": int(cmd.base),
            "elbow": int(cmd.elbow),
            "solenoid": int(self.solenoid_state)
        }
    
    """
    ------------------------------------------------------------
    Function: on_message

    Input:
        client :
            MQTT client instance.
        userdata :
            Private user data (unused).
        msg :
            Incoming MQTT message object.

    Output:
        None

    Logic Explanation:

        1. Verify that the received message belongs
           to this robot’s sensor topic.

        2. Decode MQTT payload from bytes to JSON.

        3. Extract IR sensor value from payload.

        4. Convert Active-LOW logic to ROS Bool:
           - ir == 0 → Object detected → True
           - ir == 1 → No object → False

        5. Publish converted IR status to the
           robot-specific ROS topic.

        6. Handle exceptions gracefully to prevent
           bridge failure due to malformed messages.

        7. Enables real-time hardware-to-ROS
           sensor feedback integration.

    Example:
        Automatically triggered on MQTT message reception.
    """
    def on_message(self, client, userdata, msg):
        """Handle incoming MQTT messages (IR Sensor)"""
        try:
            if msg.topic == self.sensor_topic_mqtt:
                data = json.loads(msg.payload.decode())

                # Create ROS Bool message
                ir_msg = Bool()

                # Logic: IR Sensor is Active LOW (0 means detected)
                # If ir == 0 -> Object Detected -> True
                # If ir == 1 -> Empty -> False
                val = data.get("ir")
                ir_msg.data = True if val == 0 else False

                self.ir_pub.publish(ir_msg)

        except Exception as e:
            self.get_logger().error(f"MQTT Rx Error: {e}")

"""
------------------------------------------------------------
Function: main

Input:
    None

Output:
    None

Logic Explanation:

    1. Initialize the ROS2 communication layer using rclpy.

    2. Create an instance of the MqttBridge node,
       which establishes ROS–MQTT communication.

    3. Enter the spin loop to process ROS callbacks
       (subscriptions and services).

    4. On shutdown, destroy the node instance
       and gracefully shut down ROS2.

    5. Acts as the entry point for launching
       the MQTT bridge process.

Example:
    main()
"""
def main():
    rclpy.init()
    node = MqttBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()