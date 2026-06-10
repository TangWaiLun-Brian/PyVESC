import asyncio
import serial_asyncio
import serial
import socket  
import pyvesc
from pyvesc.VESC.messages import *
from pyvesc.VESC.messages.setters import SetDutyCycle, SetCurrent, SetRPM
from pyvesc.VESC.messages.getters import GetValues, GetRotorPosition, GetIMUData, GetMcConf, GetIOBoardData, GetDecodedADC
from typing import Optional
import struct
import math
from enum import Enum, auto
import inspect
from pprint import pprint
import logging
import sys
if sys.platform == 'win32':
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    
class CommMode(Enum):
    SERIAL = auto()
    TCP = auto()
    UDP = auto()

class ControlMode(Enum):
    DUTY_CYCLE = auto()
    CURRENT = auto()
    RPM = auto()
    POSITION = auto()

class SensorType(Enum):
    IMU = auto()
    LOAD_CELL = auto()
    ADC = auto()

def decode_float32_auto(raw_uint32: int) -> float:
    """Decode a float encoded with buffer_append_float32_auto"""
    if raw_uint32 == 0:
        return 0.0
    
    sign = 1.0 if (raw_uint32 & (1 << 31)) == 0 else -1.0
    
    exponent = (raw_uint32 >> 23) & 0xFF
    mantissa_uint = raw_uint32 & 0x7FFFFF
    
    if exponent == 0:
        return 0.0  # Should not happen due to encoding, but safe
    
    # Adjust exponent back (encoded e += 126)
    exponent -= 126
    
    # Reconstruct significand: encoded sig_i = (abs(sig) - 0.5) * 2 * 8388608
    # So abs(sig) = sig_i / (2 * 8388608) + 0.5
    sig_abs = mantissa_uint / (2.0 * 8388608.0) + 0.5
    
    # Rebuild float using ldexp (same as frexp reverse)
    return sign * math.ldexp(sig_abs, exponent)

# Helper to extract the uint32 from buffer (big-endian, like VESC)
def buffer_get_float32_auto(buffer: bytes, index: int) -> float:
    raw = struct.unpack_from(">I", buffer, index)[0]  # > = big-endian
    index += 4
    return decode_float32_auto(raw)

class AsyncVESC_TCP:
    def __init__(self, host: str, port: int = 65102, commMode: CommMode = CommMode.SERIAL, request_freq: float = 20.0):
        self.host = host
        self.port = port
        self.commMode = commMode
        self.reader: Optional[asyncio.StreamReader] = None
        self.writer: Optional[asyncio.StreamWriter] = None
        self.buffer = bytearray()
        self._receive_task: Optional[asyncio.Task] = None
        self._request_task: Optional[asyncio.Task] = None
        self.request_freq = request_freq
        self._closing = False              # ← important flag
        self._closed_event = asyncio.Event()  # signal that we're done
        self._lock = asyncio.Lock()  # To protect writer during sends
        self.motors = {}  # can_id to motor mapping
        self.sensor_requests = []
        self.imu_s = {}
        self.adc_s = {}
        self.main_can_id = None
        self.mc_config_id = None

    async def connect(self):
        """Establish TCP connection and start background receive task"""
        if self.commMode == CommMode.TCP:
            self.reader, self.writer = await asyncio.open_connection(self.host, self.port)
            sock = self.writer.get_extra_info('socket')
            if sock:
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"Connected to VESC at {self.host}:{self.port} via TCP")
        elif self.commMode == CommMode.SERIAL:
            self.reader, self.writer = await serial_asyncio.open_serial_connection(
            url=self.host,              # ← this is the important part
            baudrate=115200,         # change to your device's baud rate
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
            )
            print(f"Connected to VESC at {self.host} via SERIAL")


        # Start background task for receiving and handling messages
        self._receive_task = asyncio.create_task(self._receive_loop())

    async def disconnect(self):
        """Graceful shutdown — should be called only once ideally"""
        if self._closing:
            await self._closed_event.wait()   # already closing → just wait
            return

        self._closing = True

        # 1. Cancel background tasks (safe even if already done)
        tasks_to_cancel = []
        if self._receive_task and not self._receive_task.done():
            tasks_to_cancel.append(self._receive_task)
        if self._request_task and not self._request_task.done():
            tasks_to_cancel.append(self._request_task)

        for t in tasks_to_cancel:
            t.cancel()

        # 2. Wait for them (with timeout protection)
        if tasks_to_cancel:
            done, pending = await asyncio.wait(
                tasks_to_cancel,
                timeout=2.0,           # important safety net
                return_when=asyncio.ALL_COMPLETED
            )
            for t in pending:
                print(f"Task did not finish in time: {t.get_name() or t}")
                # You can choose to t.cancel() again or just leave it

        # 3. Close transport
        if self.writer and not self.writer.is_closing():
            self.writer.close()
            try:
                await asyncio.wait_for(self.writer.wait_closed(), timeout=2.0)
            except asyncio.TimeoutError:
                print("writer.wait_closed() timed out")

        self._closed_event.set()
        print("Disconnected (clean shutdown)")

    def add_sensor2(self, device):
        device_can_id = device.can_id
        if self.main_can_id == device.can_id:
            device.can_id = None

        if device.type == SensorType.IMU:
            self.imu_s[device_can_id] = device
        elif device.type == SensorType.ADC:
            self.adc_s[device_can_id] = device
        else:
            raise NotImplementedError(f"Support for device type '{device.sensor_type}' has not been implemented yet.")

    def add_sensor(self, device_type: SensorType, can_id: int | None=None, *args, **kwargs):
        if self.main_can_id == can_id:
            can_id = None
        if device_type == SensorType.IMU:
            request_func = lambda cid, a=args, k=kwargs: self.request_imu_data(can_id=cid, *a, **k)
            self.sensor_requests.append((can_id, request_func))
        elif device_type == SensorType.ADC:
            request_func = lambda cid, a=args, k=kwargs: self.request_ioboard_data(can_id=cid, *a, **k)
            self.sensor_requests.append((can_id, request_func))
        else:
            raise NotImplementedError(f"Support for device type '{device_type}' has not been implemented yet.")
        
    def add_motor(self, motor, can_id: int, main_device: bool = False):
        """Register a motor with a specific CAN ID"""
        if main_device:
            self.main_can_id = can_id
            motor.can_id = None
        print(f"Added motor with CAN ID {can_id}")
        self.motors[can_id] = motor

    def set_motor_encoder_offsets(self):
        for motor in self.motors.values():
            motor.setEncoderOffset()

    def enable_update(self):
        """Start periodic status update requests for all devices"""
        self._request_task = asyncio.create_task(self._request_status_update())

    async def _request_status_update(self):
        try:
            while True:
                for can_id, motor in self.motors.items():
                    await motor.requestStatusUpdate()
                
                # for can_id, request_func in self.sensor_requests:
                #     await request_func(can_id)
                for can_id, imu in self.imu_s.items():
                    await imu.requestStatusUpdate()
                for can_id, adc in self.adc_s.items():
                    await adc.requestStatusUpdate()

                await asyncio.sleep(1 / self.request_freq)  # Adjust the interval as needed
        except Exception as e:
            print(f"Request loop error: {e}")
        finally:
            if not self._closing:
                # Only initiate close if nobody else is doing it
                asyncio.create_task(self.disconnect())

    async def _receive_loop(self):
        """Background task: continuously read and decode incoming packets"""
        try:
            while True:
                data = await self.reader.read(4096)
                if not data:
                    print("Connection closed by remote")
                    break

                self.buffer.extend(data)

                while self.buffer:
                    msg, consumed = pyvesc.decode(bytes(self.buffer))
                    if msg is None:
                        # Need more data
                        break
                    
                    # Handle received message
                    await self._handle_message(msg)

                    # Remove consumed bytes
                    del self.buffer[:consumed]

        except asyncio.IncompleteReadError:
            print("Connection lost (incomplete read)")
        except Exception as e:
            print(f"Receive loop error: {e}")
        finally:
            if not self._closing:
                # Only initiate close if nobody else is doing it
                asyncio.create_task(self.disconnect())

    async def _handle_message(self, msg):
        """Override or extend this method to handle incoming messages"""
        if hasattr(msg, 'app_controller_id'):
            can_id = int.from_bytes(msg.app_controller_id, 'big') if isinstance(msg.app_controller_id, (bytes, bytearray)) else msg.app_controller_id
        else:
            can_id = self.mc_config_id
            if can_id is None:
                can_id = self.main_can_id
        

        logging.info(f"← Target ID: {can_id} Received: {msg.__class__.__name__}")
        # Example handlers
        if isinstance(msg, GetValues):
            # print(f"   RPM: {msg.rpm}")
            # print(f"   Duty Cycle: {msg.duty_cycle_now:.3f}")
            # print(f"   Current: {msg.avg_motor_current:.2f}A")
            # print(f"   PID Position: {msg.pid_pos_now:.2f}°")
            # print(f"Avg id: {msg.avg_id}, iq: {msg.avg_iq}")
            # print(f"CAN ID {can_id} - RPM: {msg.rpm}, Current: {msg.avg_motor_current:.2f}A, PID Pos: {msg.pid_pos_now:.2f}°")

            self.motors[can_id].setMotorStatus(msg)

        if isinstance(msg, GetValuesSelective):
            # print(msg.avg_motor_current)
            # print(msg.rpm)
            # print(msg.pid_pos_now)
            # print(f"CAN ID {can_id} - RPM: {msg.rpm}, Current: {msg.avg_motor_current:.2f}A, PID Pos: {msg.pid_pos_now:.2f}°")
            self.motors[can_id].setMotorStatus(msg)

        # Add more as needed (e.g., GetRotorPosition, GetEncoder, etc.)
        if isinstance(msg, GetIMUData):
            rpy_x = decode_float32_auto(msg.rpy_x)
            rpy_y = decode_float32_auto(msg.rpy_y)
            rpy_z = decode_float32_auto(msg.rpy_z)
            quad_w = decode_float32_auto(msg.quad_w)
            quad_x = decode_float32_auto(msg.quad_x)
            quad_y = decode_float32_auto(msg.quad_y)
            quad_z = decode_float32_auto(msg.quad_z)
            # print(f"   Quaternion: w={quad_w:.4f}, x={quad_x:.4f}, y={quad_y:.4f}, z={quad_z:.4f}")
            self.imu_s[can_id].setStatus((rpy_x, rpy_y, rpy_z), (quad_w, quad_x, quad_y, quad_z))
        
        if isinstance(msg, GetDecodedADC):
            print(f"{msg.decoded_level:2f}, {msg.voltage:.2f}V, {msg.decoded_level2:2f}, {msg.voltage2:.2f}V")

        # Mc Config
        if isinstance(msg, GetMcConf):
            # print(f"motor current max: {msg.l_current_max} A")
            # print(f"motor current min: {msg.l_current_min} A")
            # print(f"motor in current max: {msg.l_in_current_max} A")
            # print(f"motor in current min: {msg.l_in_current_min} A")
            # print(f"motor flux linkage: {msg.foc_motor_flux_linkage} mWb")
            # print(f"foc observer gain: {msg.foc_observer_gain}")
            self.motors[can_id].setMotorConfig(msg)

    async def _send_packet(self, packet: bytes):
        async with self._lock:
            self.writer.write(packet)
            # Only drain if the buffer is getting significantly large
            if self.writer.transport.get_write_buffer_size() > 1024:
                await self.writer.drain()

    # === Synchronous-style command functions ===

    async def request_imu_data(self, mask: int = 0xF007, can_id: int | None = None):
        """
        Send COMM_GET_IMU_DATA request.
        Recommended masks:
        - 0xF000  : Quaternion (w/x/y/z)
        - 0xF007 : Quaternion (w/x/y/z) + Roll,Pitch,Yaw
        """
        # Build payload: command ID (1 byte) + mask (2 bytes big-endian)
        payload = struct.pack('>BH', VedderCmd.COMM_GET_IMU_DATA, mask)  # > = big-endian, B=uint8, H=uint16
        
        if can_id is not None:
            # For CAN forwarding: prepend COMM_FORWARD_CAN (usually 34) + can_id
            # Adjust 34 if your firmware uses a different value (rare)
            payload = struct.pack('>BB', VedderCmd.COMM_FORWARD_CAN, can_id) + payload
        
        # Encode the full VESC packet (handles start/stop bytes, length, CRC)
        packet = pyvesc.protocol.packet.codec.frame(payload)

        
        await self._send_packet(packet)
        target = f" (CAN ID {can_id})" if can_id else ""
        logging.info(f"→ Requested IMU Data (mask=0x{mask:04X}){target}")
    
    async def request_ioboard_data(self, mask: int = 0x00FF, can_id: int | None = None):
        """
        Send COMM_GET_IO_BOARD_DATA request.
        Recommended masks:

        """
        # Build payload: command ID (1 byte) + mask (2 bytes big-endian)
        payload = struct.pack('>BH', VedderCmd.COMM_IO_BOARD_GET_ALL, mask)  # > = big-endian, B=uint8, H=uint16
        
        if can_id is not None:
            # For CAN forwarding: prepend COMM_FORWARD_CAN (usually 34) + can_id
            # Adjust 34 if your firmware uses a different value (rare)
            payload = struct.pack('>BB', VedderCmd.COMM_FORWARD_CAN, can_id) + payload
        
        # Encode the full VESC packet (handles start/stop bytes, length, CRC)
        packet = pyvesc.protocol.packet.codec.frame(payload)

        
        await self._send_packet(packet)
        target = f" (CAN ID {can_id})" if can_id else " self"
        logging.info(f"→ Requested IO Board Data (mask=0x{mask:04X}){target}")

        # packet = pyvesc.encode_request(GetDecodedADC(can_id=can_id))
        # await self._send_packet(packet)
        # logging.info(f"→ Requested GetDecodedADC of can id {can_id}")


# === Example Usage ===
async def main():
    # vesc = AsyncVESC_TCP("192.168.0.146", 65102)  # Replace with your TCP bridge IP
    vesc = AsyncVESC_TCP("COM4", commMode=CommMode.SERIAL)  # Replace with your serial port
    can_id = None
    try:
        await vesc.connect()

        # Example: send position commands synchronously
        # await vesc.set_position(0.0, can_id=can_id)
        # await asyncio.sleep(2)

        # await vesc.set_position(90.0, can_id=can_id)
        # await asyncio.sleep(2)

        # await vesc.set_position(180.0, can_id=can_id)
        # await asyncio.sleep(2)

        # # Example: current control
        # await vesc.set_current(1.0)  # 0.1A
        # await asyncio.sleep(2)
        # await vesc.set_current(0.0)  # 0.1A
        # await asyncio.sleep(2)


        # # Request imu data
        # while True:
        #     await vesc.request_imu_data(mask=0xF007, can_id=None)
        #     await asyncio.sleep(0.5)

        # Get motor configuration
        await vesc.get_values(can_id=can_id)
        await vesc.get_values(can_id=21)
        await asyncio.sleep(0.2)



    except Exception as e:
        logging.error(f"Error: {e}")
    finally:
        await vesc.disconnect()

if __name__ == "__main__":
    asyncio.run(main())