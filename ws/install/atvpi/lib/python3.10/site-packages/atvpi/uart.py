import serial
import struct
import time

# Configure the serial port
uart = serial.Serial(
    port='/dev/ttyS0',  # Adjust to the correct UART port (e.g., '/dev/ttyS0' or '/dev/serial0')
    baudrate=115200,
    timeout=1
)

def send_message():
    # Define message components
    start_byte = b's'
    uint16_values = [1000, 2000, 3000, 4000]  # Example uint16_t values
    int16_values = [-100, 100]               # Example int16_t values
    end_byte = b'e'

    # Pack message using struct
    message = struct.pack('<c4H2hc', start_byte, *uint16_values, *int16_values, end_byte)

    # Send the message
    uart.write(message)

try:
    while True:
        send_message()
        print("Message sent.")
        time.sleep(1)  # Send message every 1 second
except KeyboardInterrupt:
    print("Exiting.")
finally:
    uart.close()
