import serial

def convert_baud_rate(baud_rate):
    supported_baud_rates = [
        1200, 1800, 2400, 4800, 9600,
        19200, 38400, 57600, 115200, 230400
    ]
    if baud_rate in supported_baud_rates:
        return baud_rate
    else:
        print(f"Error! Baud rate {baud_rate} not supported! Defaulting to 57600")
        return 57600


class ArduinoComms:
    def __init__(self):
        self.serial_conn = None
        self.timeout_s = 1 

    def connect(self, serial_device, baud_rate, timeout_ms):
        self.timeout_s = timeout_ms / 1000.0
        baud = convert_baud_rate(baud_rate)
        self.serial_conn = serial.Serial(
            port=serial_device,
            baudrate=baud,
            timeout=self.timeout_s
        )

    def disconnect(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()

    def connected(self):
        return self.serial_conn.is_open if self.serial_conn else False

    def send_msg(self, msg_to_send, print_output=False):
        if not self.connected():
            print("Serial port not open.")
            return ""

        self.serial_conn.reset_input_buffer()
        self.serial_conn.reset_output_buffer()

        self.serial_conn.write(msg_to_send.encode())

        try:
            response = self.serial_conn.readline().decode().strip()
        except serial.SerialTimeoutException:
            print("The readline() call has timed out.")
            response = ""

        if print_output:
            print(f"Sent: {msg_to_send.strip()} Recv: {response}")

        return response

    def send_empty_msg(self):
        self.send_msg("\r")

    def set_drive_motor_value(self, val):
        msg = f"o {val}\r"
        self.send_msg(msg)

    def set_steering_motor_value(self, val):
        msg = f"s 0 {val}\r"
        self.send_msg(msg)
