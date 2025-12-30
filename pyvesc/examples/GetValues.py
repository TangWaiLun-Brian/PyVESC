import pyvesc
from pyvesc.VESC.messages import SetRPM, SetCurrent, GetValues
import serial
import time

# Set your serial port here (either /dev/ttyX or COMX)
serialport = 'COM3'


def get_values_example():
    with serial.Serial(serialport, baudrate=115200, timeout=0.05) as ser:
        try:
            while True:
                ser.write(pyvesc.encode(SetRPM(200)))

                # Request the current measurement from the vesc
                ser.write(pyvesc.encode_request(GetValues))

                # Read all available incoming data
                while ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting)
                    try:
                        response, consumed = pyvesc.decode(data)
                        # Only print if the response is a GetValues object
                        if isinstance(response, GetValues):
                            # Attempt to extract the raw bytes for pid_pos_now from the original data
                            # This assumes pid_pos_now is at a fixed offset in the GetValues response
                            print('pid_pos_now (decoded):', response.pid_pos_now)
                            # Print the raw bytes (for debugging, may need to adjust offset/length)
                            # Example: pid_pos_now is usually a float (4 bytes), offset may vary by firmware
                            # Here we print the full data for manual inspection
                            print('Raw data (hex):', data.hex())
                    except Exception:
                        pass

                time.sleep(0.1)

        except KeyboardInterrupt:
            # Turn Off the VESC
            ser.write(pyvesc.encode(SetCurrent(0)))


if __name__ == "__main__":
    get_values_example()
