import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray, Int32
import asyncio
import websockets
import json

class GamepadWebSocketNode(Node):
    def __init__(self):
        super().__init__('gamepad_websocket_node')
        self.get_logger().info("Gamepad WebSocket Node Started")

        # ROS2 publishers
        self.axis_pub = self.create_publisher(Float32MultiArray, 'joystick/axis_values', 10)
        self.button_pub = self.create_publisher(Int32MultiArray, 'joystick/button_values', 10)
        self.mode_pub = self.create_publisher(Int32MultiArray, 'joystick/modes', 10)

        # WebSocket server details
        self.websocket_uri = "ws://atvpi.local:8000/ws/ros"  # Replace with your RPi's IP
        self.loop = asyncio.get_event_loop()

        # Variables
        self.modes = [0,0]  # 0 or 1
        self.prev_button_1_state = 0
        self.prev_button_2_state = 0  # To detect button press changes


        # Start WebSocket client
        self.loop.run_until_complete(self.connect_websocket())

    async def connect_websocket(self):
        """Connect to the WebSocket server and listen for gamepad data."""
        async with websockets.connect(self.websocket_uri) as websocket:
            self.get_logger().info("Connected to WebSocket server")
            while 1:
                try:
                    # Receive gamepad data
                    data = await websocket.recv()
                    gamepad_data = json.loads(data)
                    self.get_logger().info(f"Gamepad Data: {gamepad_data}")
                    self.publish_data(gamepad_data)                
                except Exception as e:
                    self.get_logger().error(f"WebSocket error: {e}")
                    break

    def publish_data(self, data: dict):
        # Get axis values
        axis_values = Float32MultiArray()
        axis_values.data = [float(i) for i in data['axes']]

        # Get button values
        button_values = Int32MultiArray()
        button_values.data = [int(i) for i in data['buttons']]


        # Check for mode toggle on button[0]
        button_1_state = button_values.data[3] # check
        if button_1_state == 1 and self.prev_button_1_state == 0:  # Detect rising edge
            self.modes[0] = 1 - self.modes[0]  # Toggle between 0 and 1

        button_2_state = button_values.data[1] # check
        if button_2_state == 1 and self.prev_button_2_state == 0:  # Detect rising edge
            self.modes[1] = 1 - self.modes[1]  # Toggle between 0 and 1

        self.prev_button_1_state = button_1_state
        self.prev_button_2_state = button_2_state

        # Publish data
        self.axis_pub.publish(axis_values)
        self.button_pub.publish(button_values)

        mode_values = Int32MultiArray()
        mode_values.data = self.modes
        self.mode_pub.publish(mode_values)

        return


def main(args=None):
    rclpy.init(args=args)
    node = GamepadWebSocketNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()