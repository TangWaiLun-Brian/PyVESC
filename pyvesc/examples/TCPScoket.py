import asyncio
import serial_asyncio
import serial
import pyvesc
from pyvesc.VESC.messages import *
from pyvesc.VESC.messages.setters import SetDutyCycle, SetCurrent, SetRPM
from pyvesc.VESC.messages.getters import GetValues, GetRotorPosition, GetIMUData, GetMcConf
from typing import Optional
import struct
import math
from enum import Enum, auto

class CommMode(Enum):
    SERIAL = auto()
    TCP = auto()
    UDP = auto()

class ControlMode(Enum):
    DUTY_CYCLE = auto()
    CURRENT = auto()
    RPM = auto()
    POSITION = auto()


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
    def __init__(self, host: str, port: int = 65102, commMode: CommMode = CommMode.SERIAL):
        self.host = host
        self.port = port
        self.commMode = commMode
        self.reader: Optional[asyncio.StreamReader] = None
        self.writer: Optional[asyncio.StreamWriter] = None
        self.buffer = bytearray()
        self._receive_task: Optional[asyncio.Task] = None
        self._lock = asyncio.Lock()  # To protect writer during sends

    async def connect(self):
        """Establish TCP connection and start background receive task"""
        if self.commMode == CommMode.TCP:
            self.reader, self.writer = await asyncio.open_connection(self.host, self.port)
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
        """Close connection gracefully"""
        if self._receive_task:
            self._receive_task.cancel()
            try:
                await self._receive_task
            except asyncio.CancelledError:
                pass

        if self.writer:
            self.writer.close()
            await self.writer.wait_closed()
        print("Disconnected")

    async def _receive_loop(self):
        """Background task: continuously read and decode incoming packets"""
        try:
            while True:
                data = await self.reader.read(100)
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
            await self.disconnect()

    async def _handle_message(self, msg):
        """Override or extend this method to handle incoming messages"""
        print(f"← Received: {msg.__class__.__name__}")

        # Example handlers
        if isinstance(msg, GetValues):
            print(f"   RPM: {msg.rpm}")
            print(f"   Duty Cycle: {msg.duty_cycle_now:.3f}")
            print(f"   Current: {msg.avg_motor_current:.2f}A")
            print(f"   PID Position: {msg.pid_pos_now:.2f}°")
        # Add more as needed (e.g., GetRotorPosition, GetEncoder, etc.)
        if isinstance(msg, GetIMUData):
            quad_w = decode_float32_auto(msg.quad_w)
            quad_x = decode_float32_auto(msg.quad_x)
            quad_y = decode_float32_auto(msg.quad_y)
            quad_z = decode_float32_auto(msg.quad_z)
            print(f"   Quaternion: w={quad_w:.4f}, x={quad_x:.4f}, y={quad_y:.4f}, z={quad_z:.4f}")
            
        # Mc Config
        if isinstance(msg, GetMcConf):
            print(f"motor current max: {msg.l_current_max} A")
            print(f"motor current min: {msg.l_current_min} A")
            print(f"motor in current max: {msg.l_in_current_max} A")
            print(f"motor in current min: {msg.l_in_current_min} A")
            print(f"motor flux linkage: {msg.foc_motor_flux_linkage} mWb")
            print(f"foc observer gain: {msg.foc_observer_gain}")

    async def _send_packet(self, packet: bytes):
        """Internal: send raw packet with lock to prevent interleaving"""
        async with self._lock:
            self.writer.write(packet)
            await self.writer.drain()

    # === Synchronous-style command functions ===

    async def set_position(self, degrees: float, **kwargs):
        """Send COMM_SET_POS (position control)"""
        packet = pyvesc.encode(SetPosition(degrees, **kwargs))
        await self._send_packet(packet)
        print(f"→ Sent SetPos: {degrees}°")

    async def set_duty_cycle(self, duty: float, **kwargs):
        """Duty cycle from -1.0 to 1.0"""
        packet = pyvesc.encode(SetDutyCycle(duty))
        await self._send_packet(packet)
        print(f"→ Sent SetDutyCycle: {duty:.3f}")

    async def set_current(self, current_amps: float, **kwargs):
        packet = pyvesc.encode(SetCurrent(current_amps, **kwargs))
        await self._send_packet(packet)
        print(f"→ Sent SetCurrent: {current_amps:.2f}A")

    async def set_rpm(self, rpm: int):
        packet = pyvesc.encode(SetRPM(rpm))
        await self._send_packet(packet)
        print(f"→ Sent SetRPM: {rpm}")

    async def request_values(self):
        """Request full GetValues struct"""
        packet = pyvesc.encode_request(GetValues(can_id=117))
        await self._send_packet(packet)
        print("→ Requested GetValues")

    async def get_motor_config(self, **kwargs):
        """Send COMM_GET_MCCONF to request motor configuration"""
        packet = pyvesc.encode_request(GetMcConf(**kwargs))
        await self._send_packet(packet)
        print("→ Requested Motor Configuration (COMM_GET_MCCONF)")

    async def request_imu_data(self, mask: int = 0xFFFF, can_id: int | None = None):
        """
        Send COMM_GET_IMU_DATA request.
        Recommended masks:
        - 0x3FF  : Roll, pitch, yaw + accel (x/y/z) + gyro (x/y/z)  → most common (9 floats)
        - 0xFFFF : Everything (including quaternion if firmware supports it)
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
        print(f"→ Requested IMU Data (mask=0x{mask:04X}){target}")
# === Example Usage ===
async def main():
    # vesc = AsyncVESC_TCP("192.168.0.146", 65102)  # Replace with your TCP bridge IP
    vesc = AsyncVESC_TCP("COM4", commMode=CommMode.SERIAL)  # Replace with your serial port
    can_id = None
    try:
        await vesc.connect()

        # # Example: send position commands synchronously
        # await vesc.set_position(0.0, can_id=can_id)
        # await asyncio.sleep(2)

        # await vesc.set_position(90.0, can_id=can_id)
        # await asyncio.sleep(2)

        # await vesc.set_position(180.0, can_id=can_id)
        # await asyncio.sleep(2)

        # Example: current control
        await vesc.set_current(1.0)  # 0.1A
        await asyncio.sleep(2)
        await vesc.set_current(0.0)  # 0.1A
        await asyncio.sleep(2)


        # # Request imu data
        # while True:
        #     await vesc.request_imu_data(mask=0xF000, can_id=117)
        #     await asyncio.sleep(0.5)

        # Get motor configuration
        await vesc.get_motor_config(can_id=can_id)
        await asyncio.sleep(2)



    except Exception as e:
        print(f"Error: {e}")
    finally:
        await vesc.disconnect()

if __name__ == "__main__":
    asyncio.run(main())