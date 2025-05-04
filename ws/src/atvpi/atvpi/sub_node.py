import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray, Int32
import serial
import math
import struct

class JoystickSubscriber(Node):
    def __init__(self):
        super().__init__('joystick_subscriber')

        # Initialize UART
        self.uart = serial.Serial('/dev/ttyS0', baudrate=115200, timeout=1)  # Adjust port and baudrate as needed

        # Subscribers
        self.create_subscription(Float32MultiArray, 'joystick/axis_values', self.axis_callback, 10)
        self.create_subscription(Int32MultiArray, 'joystick/button_values', self.button_callback, 10)
        self.create_subscription(Int32, 'joystick/mode', self.mode_callback, 10)

        # Variables
        self.prev_button_1_state = 0  # To detect rising edge
        self.mode = 0
        self.axes = [0]*4
        self.buttons = [0]*12
        self.servo_values = [3050,3050,3050,3050]  # Example uint16_t values
        self.dc_values = [0,0]               # Example int16_t values

        # Timer callback
        self.timer = self.create_timer(0.1, self.timer_callback)

    def axis_callback(self, msg):
        self.axes = [round(i,2) for i in msg.data] # filter values
        #print(self.axes)

    def button_callback(self, msg):
        self.buttons = msg.data

    def mode_callback(self, msg):
        # Mode changes
        self.mode = msg.data

    def send_message(self): # send message over serial
        # Define message components
        start_byte = b's'
        end_byte = b'e'

        uint16_values = [int(i) for i in self.servo_values]
        int16_values = [int(i) for i in self.dc_values]
        
        # Pack message using struct
        message = struct.pack('<c4H2hc', start_byte, *uint16_values, *int16_values, end_byte)

        # Send the message
        self.uart.write(message)


    def timer_callback(self):
        # process raw joystick values
        
        w = 23 # wheel center to vehicle center width
        l = 16.2 # wheel center to vehicle center length
        servo_range = 160/180*math.pi # need to calculate, in rad

        if(self.mode == 0): # ackermann
            steer = self.axes[3] 
            throttle = -self.axes[1]*2000
            
            if (steer == 0): # no steer input
                R = 0
                self.servo_values = [3050]*4
                self.dc_values = [throttle]*2
            else:
                R = 1.5*w/steer # chosen arbitrarily
                thetaL = 4800.0/servo_range*math.atan(l/(R+w))
                thetaR = 4800.0/servo_range*math.atan(l/(R-w))

                self.servo_values[0] = 3050 - thetaL # FL
                self.servo_values[1] = 3050 - thetaR # FR
                self.servo_values[2] = 3050 + thetaL # BR
                self.servo_values[3] = 3050 + thetaR # BL

                self.dc_values = [throttle*((l**2 + (R+w)**2)**0.5/abs(R))*(1-0.5*abs(steer)),throttle*((l**2 + (R-w)**2)**0.5/abs(R))*(1-0.5*abs(steer))] # R, L

        elif(self.mode == 1): # differential (on the spot)
            throttle = self.axes[3]*1500 # limit max duty

            theta = 4800.0/servo_range*math.atan(l/w)

            # servo angles
            self.servo_values[0] = 3050 - theta # FL
            self.servo_values[1] = 3050 + theta # FR
            self.servo_values[2] = 3050 + theta # BR
            self.servo_values[3] = 3050 - theta # BL

            self.dc_values = [throttle, -throttle] # L, R


        
        self.send_message()
        #self.get_logger().info(f"Axis: {self.axes}, Buttons: {self.buttons}, Mode: {self.mode}")
        # self.get_logger().info(f"Mode: {self.mode}, Servos: {self.servo_values}, DC: {self.dc_values}")



def main(args=None):
    rclpy.init(args=args)
    node = JoystickSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


