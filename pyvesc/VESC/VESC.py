from pyvesc.protocol.interface import encode_request, encode, decode
from pyvesc.VESC.messages import *
import time
import threading

# because people may want to use this library for their own messaging, do not make this a required package
try:
    import serial
except ImportError:
    serial = None


class VESC(object):
    def __init__(self, serial_port, has_sensor=False, start_heartbeat=False, baudrate=115200, timeout=None):
        """
        :param serial_port: Serial device to use for communication (i.e. "COM3" or "/dev/tty.usbmodem0")
        :param has_sensor: Whether or not the bldc motor is using a hall effect sensor
        :param start_heartbeat: Whether or not to automatically start the heartbeat thread that will keep commands
                                alive.
        :param baudrate: baudrate for the serial communication. Shouldn't need to change this.
        :param timeout: timeout for the serial communication
        """

        if serial is None:
            raise ImportError("Need to install pyserial in order to use the VESCMotor class.")

        self.serial_port = serial.Serial(port=serial_port, baudrate=baudrate, write_timeout=0) # changed timeout=timeout to write_timeout=0
        if has_sensor:
            self.serial_port.write(encode(SetRotorPositionMode(SetRotorPositionMode.DISP_POS_OFF)))

        self.alive_msg = [encode(Alive())]

        self.heart_beat_thread = threading.Thread(target=self._heartbeat_cmd_func)
        self._stop_heartbeat = threading.Event()

        if start_heartbeat:
            self.start_heartbeat()

        # check firmware version and set GetValue fields to old values if pre version 3.xx
        while True:
            try:
                version = self.get_firmware_version()
                if int(version.split('.')[0]) < 3:
                    GetValues.fields = pre_v3_33_fields
                break
            except Exception as e:
                print(f"Error getting firmware version: {e}")
                print("Retrying ...")
                time.sleep(0.1)

        # store message info for getting values so it doesn't need to calculate it every time
        msg = GetValues()
        self._get_values_msg = encode_request(msg)
        self._get_values_msg_expected_length = msg._full_msg_size

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop_heartbeat()
        if self.serial_port.is_open:
            self.serial_port.flush()
            self.serial_port.close()

    def _heartbeat_cmd_func(self):
        """
        Continuous function calling that keeps the motor alive
        """
        while not self._stop_heartbeat.is_set():
            time.sleep(0.1)
            for i in self.alive_msg:
                self.write(i)

    def start_heartbeat(self, can_id=None):
        """
        Starts a repetitive calling of the last set cmd to keep the motor alive.

        Args:
            can_id: Optional, used to specify the CAN ID to add to the existing heartbeat messaged
        """
        if can_id is not None:
            self.alive_msg.append(encode(Alive(can_id=can_id)))
        else:
            self.heart_beat_thread.start()

    def stop_heartbeat(self):
        """
        Stops the heartbeat thread and resets the last cmd function. THIS MUST BE CALLED BEFORE THE OBJECT GOES OUT OF
        SCOPE UNLESS WRAPPING IN A WITH STATEMENT (Assuming the heartbeat was started).
        """
        self._stop_heartbeat.set()
        if self.heart_beat_thread.is_alive():
            self.heart_beat_thread.join()

    def write(self, data, num_read_bytes=None):
        """
        A write wrapper function implemented like this to try and make it easier to incorporate other communication
        methods than UART in the future.
        :param data: the byte string to be sent
        :param num_read_bytes: number of bytes to read for decoding response
        :return: decoded response from buffer
        """
        """
        Patched write wrapper to handle VESC FW 6.x long packets.
        Bypasses the in_waiting deadlock by stream-reading the exact packet size.
        """
        self.serial_port.reset_input_buffer()
        self.serial_port.write(data)
        
        if num_read_bytes is not None:
            # 1. Read the very first byte (blocking) to see packet type
            start_byte = self.serial_port.read(1)
            
            if not start_byte:
                print("VESC Timeout: No response received.")
                return None
                
            # 2. Read length based on packet type
            if start_byte == b'\x02':
                # Short packet
                length_bytes = self.serial_port.read(1)
                payload_len = length_bytes[0]
            elif start_byte == b'\x03':
                # Long packet (FW 6.x)
                length_bytes = self.serial_port.read(2)
                payload_len = (length_bytes[0] << 8) | length_bytes[1]
            else:
                print(f"VESC Error: Invalid start byte: {start_byte}")
                # Flush the corrupted buffer
                self.serial_port.reset_input_buffer() 
                return None
                
            # 3. Calculate remaining bytes (payload + 2-byte CRC + 1-byte stop)
            bytes_remaining = payload_len + 3
            
            # 4. Read the rest of the packet. 
            # Because this is a single .read(), PySerial handles pulling data
            # out of the OS buffer as it arrives, preventing the 490-byte lockup!
            rest_of_packet = self.serial_port.read(bytes_remaining)
            
            if len(rest_of_packet) < bytes_remaining:
                print("VESC Error: Packet timed out before finishing.")
                return None
                
            # 5. Reassemble and decode
            full_packet = start_byte + length_bytes + rest_of_packet
            response, consumed = decode(full_packet)
            
            return response

    def set_rpm(self, new_rpm, **kwargs):
        """
        Set the electronic RPM value (a.k.a. the RPM value of the stator)
        :param new_rpm: new rpm value
        """
        self.write(encode(SetRPM(new_rpm, **kwargs)))

    def set_current(self, new_current, **kwargs):
        """
        :param new_current: new current in milli-amps for the motor
        """
        self.write(encode(SetCurrent(new_current, **kwargs)))

    def set_duty_cycle(self, new_duty_cycle, **kwargs):
        """
        :param new_duty_cycle: Value of duty cycle to be set (range [-1e5, 1e5]).
        """
        self.write(encode(SetDutyCycle(new_duty_cycle, **kwargs)))

    def set_servo(self, new_servo_pos, **kwargs):
        """
        :param new_servo_pos: New servo position. valid range [0, 1]
        """
        self.write(encode(SetServoPosition(new_servo_pos, **kwargs)))

    def get_measurements(self, **kwargs):
        """
        :return: A msg object with attributes containing the measurement values
        """
        return self.write(encode_request(GetValues(**kwargs)), num_read_bytes=self._get_values_msg_expected_length)

    def get_firmware_version(self):
        msg = GetVersion()
        return str(self.write(encode_request(msg), num_read_bytes=msg._full_msg_size))
    
    def set_pos(self, new_pos, **kwargs):
        """
        :param new_pos: New  position. valid range [0, 360.0]
        """
        self.write(encode(SetPosition(new_pos, **kwargs)))

    def get_rpm(self):
        """
        :return: Current motor rpm
        """
        return self.get_measurements().rpm
    
    def get_pos(self, **kwargs):
        """
        :return: Current motor position
        """
        return self.get_measurements(**kwargs).pid_pos_now

    def get_duty_cycle(self):
        """
        :return: Current applied duty-cycle
        """
        return self.get_measurements().duty_now

    def get_v_in(self):
        """
        :return: Current input voltage
        """
        return self.get_measurements().v_in

    def get_motor_current(self):
        """
        :return: Current motor current
        """
        return self.get_measurements().current_motor

    def get_incoming_current(self):
        """
        :return: Current incoming current
        """
        return self.get_measurements().current_in
    
    def get_mc_conf(self, **kwargs):
        """
        :return: Current motor configuration
        """
        msg = GetMcConf(**kwargs)
        response = self.write(encode_request(msg), num_read_bytes=msg._full_msg_size)
        return response

    def set_mc_conf(self, mc_conf, **kwargs):
        """
        :param mc_conf: New motor configuration
        """
        field_names = [field[0] for field in SetMcConf.fields]
        if isinstance(mc_conf, dict):
            values = [mc_conf[name] for name in field_names]
        else:
            values = [getattr(mc_conf, name) for name in field_names]
            
        conf = SetMcConf(*values, **kwargs)
        bs = encode(conf)
        print(f"conf: {conf}, length: {len(bs)}, full_msg_size: {conf._full_msg_size}")
        print(f"pid_kp: {conf.p_pid_kp}")
        print(f"can_id: {conf.can_id}")

        self.write(bs)



