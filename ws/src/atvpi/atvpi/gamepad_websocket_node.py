import rclpy
from rclpy.node import Node
import asyncio
import websockets
import json

class GamepadWebSocketNode(Node):
    def __init__(self):
        super().__init__('gamepad_websocket_node')
        self.get_logger().info("Gamepad WebSocket Node Started")

        # WebSocket server details
        self.websocket_uri = "ws://atvpi.local:8000/ws/gamepad"  # Replace with your RPi's IP
        self.loop = asyncio.get_event_loop()

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
                except Exception as e:
                    self.get_logger().error(f"WebSocket error: {e}")
                    break

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