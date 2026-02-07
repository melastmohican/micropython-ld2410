import struct
import time
from micropython import const

# Constants
_REPORT_HEADER = b'\xf4\xf3\xf2\xf1'
_REPORT_FOOTER = b'\xf8\xf7\xf6\xf5'
_CMD_HEADER = b'\xfd\xfc\xfb\xfa'
_CMD_FOOTER = b'\x04\x03\x02\x01'

_INNER_HEAD = const(0xAA)
_INNER_TAIL = const(0x55)
_INNER_CHECK = const(0x00)

_MAX_REPORT_DATA_LEN = const(128)
_MAX_CMD_DATA_LEN = const(128)

# Command Words
_CMD_ENABLE_CONFIG = const(0x00FF)
_CMD_END_CONFIG = const(0x00FE)
_CMD_SET_MAX_GATES = const(0x0060)
_CMD_READ_PARAMETERS = const(0x0061)
_CMD_ENABLE_ENGINEERING = const(0x0062)
_CMD_DISABLE_ENGINEERING = const(0x0063)
_CMD_SET_SENSITIVITY = const(0x0064)
_CMD_READ_FIRMWARE = const(0x00A0)
_CMD_SET_BAUD_RATE = const(0x00A1)
_CMD_FACTORY_RESET = const(0x00A2)
_CMD_RESTART = const(0x00A3)

class TargetState:
    NO_TARGET = const(0x00)
    MOVING = const(0x01)
    STATIONARY = const(0x02)
    BOTH = const(0x03)

class LD2410Error(Exception):
    pass

class InvalidFrameError(LD2410Error):
    pass

class InvalidReportError(LD2410Error):
    pass

class InvalidAckError(LD2410Error):
    pass

class BufferOverflowError(LD2410Error):
    pass

class PresenceData:
    def __init__(self, target_state, moving_distance_cm, moving_energy, 
                 still_distance_cm, still_energy, detection_distance_cm):
        self.target_state = target_state
        self.moving_distance_cm = moving_distance_cm
        self.moving_energy = moving_energy
        self.still_distance_cm = still_distance_cm
        self.still_energy = still_energy
        self.detection_distance_cm = detection_distance_cm

    def presence(self):
        return self.target_state != TargetState.NO_TARGET

    def __repr__(self):
        state_str = "None"
        if self.target_state == TargetState.MOVING: state_str = "Moving"
        elif self.target_state == TargetState.STATIONARY: state_str = "Stationary"
        elif self.target_state == TargetState.BOTH: state_str = "Both"
        
        return (f"<PresenceData state={state_str} "
                f"mov_dist={self.moving_distance_cm}cm mov_eng={self.moving_energy} "
                f"stat_dist={self.still_distance_cm}cm stat_eng={self.still_energy} "
                f"det_dist={self.detection_distance_cm}cm>")

class LD2410:
    def __init__(self, uart):
        self.uart = uart
        self._buf = bytearray(256)

    def read_presence(self):
        """
        Read the next normal-mode presence report (type 0x02).
        Skips engineering reports and other data.
        Blocks until a valid frame is received.
        """
        while True:
            frame_data = self._read_report_frame_data()
            if len(frame_data) < 4:
                continue
            
            frame_type = frame_data[0]
            if frame_type != 0x02:
                continue
            
            return self._parse_normal_report(frame_data)

    def enter_config_mode(self):
        payload = b'\x01\x00'
        self._send_command(_CMD_ENABLE_CONFIG, payload)
        ack = self._read_ack()
        if ack['status'] != 0:
            raise InvalidAckError("Status not 0")

    def exit_config_mode(self):
        self._send_command(_CMD_END_CONFIG, b'')
        ack = self._read_ack()
        if ack['status'] != 0:
            raise InvalidAckError("Status not 0")

    def enable_engineering_mode(self):
        self._send_command(_CMD_ENABLE_ENGINEERING, b'')
        ack = self._read_ack()
        if ack['status'] != 0:
            raise InvalidAckError("Status not 0")

    def disable_engineering_mode(self):
        self._send_command(_CMD_DISABLE_ENGINEERING, b'')
        ack = self._read_ack()
        if ack['status'] != 0:
            raise InvalidAckError("Status not 0")

    def read_firmware_version(self):
        """Returns raw firmware version bytes (8 bytes typically)."""
        self._send_command(_CMD_READ_FIRMWARE, b'')
        ack = self._read_ack()
        if ack['status'] != 0:
            raise InvalidAckError("Status not 0")
        if len(ack['payload']) < 8:
             raise InvalidAckError("Payload too short")
        return ack['payload'][:8]

    def restart(self):
        self._send_command(_CMD_RESTART, b'')
        ack = self._read_ack()
        if ack['status'] != 0:
            raise InvalidAckError("Status not 0")

    def _send_command(self, cmd_word, payload):
        if len(payload) > _MAX_CMD_DATA_LEN:
            raise BufferOverflowError()

        intra_len = 2 + len(payload)
        
        # Write header
        self.uart.write(_CMD_HEADER)
        # Write length
        self.uart.write(struct.pack('<H', intra_len))
        # Write cmd
        self.uart.write(struct.pack('<H', cmd_word))
        # Write payload
        if payload:
            self.uart.write(payload)
        # Write footer
        self.uart.write(_CMD_FOOTER)

    def _read_ack(self):
        frame_len = self._read_cmd_frame_into_buf()
        buf = memoryview(self._buf)[:frame_len]
        
        # Layout: [hdr4][len2][intra...][ftr4]
        if frame_len < 14: # 4+2+2+2+4 minimum
             raise InvalidAckError("Frame too short")
             
        intra_len = struct.unpack('<H', buf[4:6])[0]
        intra_start = 6
        intra_end = intra_start + intra_len
        
        if intra_end + 4 != frame_len:
            raise InvalidAckError("Length mismatch")
            
        ack_cmd = struct.unpack('<H', buf[intra_start:intra_start+2])[0]
        status = struct.unpack('<H', buf[intra_start+2:intra_start+4])[0]
        payload = bytes(buf[intra_start+4:intra_end])
        
        return {'ack_cmd': ack_cmd, 'status': status, 'payload': payload}

    def _read_report_frame_data(self):
        total_len = self._read_report_frame_into_buf()
        # Layout: [hdr4][len2][frame_data...][ftr4]
        if total_len < 10:
             raise InvalidFrameError()
             
        data_len = struct.unpack('<H', self._buf[4:6])[0]
        data_start = 6
        data_end = data_start + data_len
        
        if data_end + 4 != total_len:
             raise InvalidFrameError()
             
        return memoryview(self._buf)[data_start:data_end]

    def _read_report_frame_into_buf(self):
        self._scan_for_header(_REPORT_HEADER)
        # Header is in buf[0:4]
        
        # Read length (2 bytes)
        self._read_exact_into(4, 2)
        data_len = struct.unpack('<H', self._buf[4:6])[0]
        
        if data_len == 0 or data_len > _MAX_REPORT_DATA_LEN:
            raise InvalidFrameError("Invalid data length")
            
        total_len = 6 + data_len + 4
        if total_len > len(self._buf):
            raise BufferOverflowError()
            
        # Read frame data + footer
        self._read_exact_into(6, data_len + 4)
        
        # Validate footer
        footer = self._buf[6+data_len : 6+data_len+4]
        if footer != _REPORT_FOOTER:
            raise InvalidFrameError("Invalid footer")
            
        return total_len

    def _read_cmd_frame_into_buf(self):
        self._scan_for_header(_CMD_HEADER)
        self._read_exact_into(4, 2)
        intra_len = struct.unpack('<H', self._buf[4:6])[0]
        
        if intra_len == 0 or intra_len > _MAX_CMD_DATA_LEN:
             raise InvalidFrameError("Invalid intra length")
             
        total_len = 6 + intra_len + 4
        if total_len > len(self._buf):
             raise BufferOverflowError()
             
        self._read_exact_into(6, intra_len + 4)
        
        footer = self._buf[6+intra_len : 6+intra_len+4]
        if footer != _CMD_FOOTER:
             raise InvalidFrameError("Invalid footer")
             
        return total_len

    def _scan_for_header(self, header):
        # Rolling match
        idx = 0
        while idx < 4:
            b = self.uart.read(1)
            if b is None: 
                # Blocking read should not return None, but strictly speaking 
                # some uPy ports might if non-blocking. 
                # Assuming blocking UART here.
                continue 
            
            byte = b[0]
            if byte == header[idx]:
                self._buf[idx] = byte
                idx += 1
            else:
                if byte == header[0]:
                    self._buf[0] = byte
                    idx = 1
                else:
                    idx = 0

    def _read_exact_into(self, offset, length):
        view = memoryview(self._buf)
        read_count = 0
        while read_count < length:
            b = self.uart.read(length - read_count)
            if b:
                view[offset + read_count : offset + read_count + len(b)] = b
                read_count += len(b)
            # else: loop until data arrives (blocking)

    def _parse_normal_report(self, frame_data):
        if len(frame_data) < 6:
            raise InvalidReportError("Too short")
        if frame_data[0] != 0x02:
             raise InvalidReportError("Wrong type")
        if frame_data[1] != _INNER_HEAD:
             raise InvalidReportError("No inner head")
        if frame_data[-2] != _INNER_TAIL:
             raise InvalidReportError("No inner tail")
        if frame_data[-1] != _INNER_CHECK:
             raise InvalidReportError("No inner check")
             
        payload = frame_data[2:-2]
        
        # Need at least target_state(1) + mov_dist(2) + mov_eng(1) + still_dist(1) + still_eng(1) + det_dist(1) = 7 bytes
        # The Rust code says: 1 + 2 + 1 + 1 + 1 + 1 = 7 bytes minimum logic check
        if len(payload) < 7:
             raise InvalidReportError("Payload too short")
             
        target_state = payload[0]
        moving_distance_cm = struct.unpack('<H', payload[1:3])[0]
        moving_energy = payload[3]
        
        idx = 4
        still_distance_cm = payload[idx]
        idx += 1
        still_energy = payload[idx]
        idx += 1
        
        # Detection distance logic from Rust
        if (idx + 1) < len(payload) and payload[idx+1] == 0x00:
             detection_distance_cm = struct.unpack('<H', payload[idx:idx+2])[0]
        else:
             detection_distance_cm = payload[idx]
             
        return PresenceData(
            target_state,
            moving_distance_cm,
            moving_energy,
            still_distance_cm,
            still_energy,
            detection_distance_cm
        )
