import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray, Int32
import serial
import math
import struct

class JoystickSubscriber(Node):
    def __init__(self):
        super().__init__('arm_controller')

        # Initialize UART
        # self.uart = serial.Serial('/dev/ttyS1', baudrate=115200, timeout=1)  # which GPIOs?

        # Subscribers
        self.create_subscription(Float32MultiArray, 'joystick/axis_values', self.axis_callback, 10)
        self.create_subscription(Int32MultiArray, 'joystick/button_values', self.button_callback, 10)
        self.create_subscription(Int32MultiArray, 'joystick/modes', self.mode_callback, 10)

        # Variables
        self.modes = [0,0]
        self.axes = [0]*4
        self.buttons = [0]*12
        
        self.node = 0 # 0, 1, 2 (0 = master)
        self.theta_target = 0
        self.phi_target = 0

        self.prev_button_up_state = 0 # to go up the arm (right trigger)
        self.prev_button_down_state = 0 # left trigger

        # Timer callback
        self.timer = self.create_timer(0.1, self.timer_callback)

    def axis_callback(self, msg):
        self.axes = [round(i,2) for i in msg.data] # filter values
        #print(self.axes)

    def button_callback(self, msg):
        self.buttons = msg.data
        
        button_up_state = self.buttons[5]
        if button_up_state == 1 and self.prev_button_up_state == 0:  # Detect rising edge
            self.node = min(self.node+1, 2)
        self.prev_button_up_state = button_up_state
        
        button_down_state = self.buttons[4]
        if button_down_state == 1 and self.prev_button_down_state == 0:  # Detect rising edge
            self.node = max(self.node-1, 0)
        self.prev_button_down_state = button_down_state

    def mode_callback(self, msg):
        # Mode changes
        self.modes = msg.data

    def send_message(self): # send message over serial
        # Define message components
        start_byte = b's'
        end_byte = b'e'

        uint16_values = [self.node]
        int16_values = [self.theta_target, self.phi_target]
        
        # Pack message using struct
        message = struct.pack('<c1H2hc', start_byte, *uint16_values, *int16_values, end_byte)

        # Send the message
        # self.uart.write(message)


    def timer_callback(self):
        if(self.modes[0] == 1): # arm mode
            self.theta_target = -self.axes[1]*2048/90*6
            self.phi_target = self.axes[2]*2048/90*6 
        else:
            self.node = 0 # REMOVE??
            self.theta_target = 0
            self.phi_target = 0
            
        # self.send_message() # should messages be sent all the time?
        self.get_logger().info(f"Axis: {self.axes}, Buttons: {self.buttons}, Modes: {self.modes}")
        self.get_logger().info(f"Node: {self.node}, Theta: {self.theta_target}, Phi: {self.phi_target}")



def main(args=None):
    rclpy.init(args=args)
    node = JoystickSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()