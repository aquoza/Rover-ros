#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import websockets
import asyncio
import numpy as np
import cv2
import threading
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class WebSocketImageSender(Node):
    def __init__(self):
        super().__init__('websocket_image_sender')
        
        # WebSocket configuration
        self.websocket_uri = "ws://atvpi.local:8000/ws/thermal"
        self.connected = False
        self.websocket = None
        
        # Image configuration
        self.bridge = CvBridge()
        self.image_width = 640
        self.image_height = 480
        
        # Create a thread for the asyncio event loop
        self.thread = threading.Thread(target=self.run_asyncio)
        self.thread.daemon = True
        self.thread.start()
        
        # Create a timer to send images periodically
        self.timer = self.create_timer(0.2, self.send_image)
        
        self.get_logger().info("WebSocket Image Sender node initialized")
    
    def run_asyncio(self):
        # Create a new event loop for the thread
        asyncio.set_event_loop(asyncio.new_event_loop())
        
        # Run the connection task
        asyncio.get_event_loop().run_until_complete(self.connect_websocket())
    
    async def connect_websocket(self):
        while rclpy.ok():
            try:
                self.get_logger().info(f"Connecting to WebSocket at {self.websocket_uri}...")
                self.websocket = await websockets.connect(self.websocket_uri)
                self.connected = True
                self.get_logger().info("WebSocket connection established")
                
                # Keep the connection alive
                while self.connected:
                    await asyncio.sleep(1)
                    
            except Exception as e:
                self.get_logger().error(f"WebSocket connection failed: {str(e)}")
                self.connected = False
                await asyncio.sleep(1)  # Wait before reconnecting
    
    def send_image(self):
        if not self.connected or self.websocket is None:
            self.get_logger().warn("Not connected to WebSocket server")
            return
        
        try:
            # Create a test image (in a real app, you'd get this from a camera or topic)
            image = np.random.randint(
                0, 256, 
                (self.image_height, self.image_width, 3), 
                dtype=np.uint8
            )
            
            # Encode as JPEG
            _, jpeg_data = cv2.imencode('.jpg', image)
            
            # Send via WebSocket (using threadsafe asyncio call)
            asyncio.run_coroutine_threadsafe(
                self._send_websocket_data(jpeg_data.tobytes()),
                asyncio.get_event_loop()
            )
            
            self.get_logger().debug(f"Sent image ({len(jpeg_data)} bytes)", throttle_duration_sec=1)
            
        except Exception as e:
            self.get_logger().error(f"Error sending image: {str(e)}")
    
    async def _send_websocket_data(self, data):
        try:
            if self.connected and self.websocket is not None:
                await self.websocket.send(data)
        except Exception as e:
            self.get_logger().error(f"WebSocket send error: {str(e)}")
            self.connected = False
            if self.websocket is not None:
                await self.websocket.close()
                self.websocket = None

def main(args=None):
    rclpy.init(args=args)
    
    node = WebSocketImageSender()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        if node.websocket is not None:
            asyncio.get_event_loop().run_until_complete(node.websocket.close())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()