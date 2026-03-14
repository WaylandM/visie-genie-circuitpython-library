import time

GENIE_ACK = 0x06
GENIE_NAK = 0x15

GENIE_WRITE_OBJ = 0x01
GENIE_READ_OBJ = 0x00
GENIE_WRITE_STR = 0x02

GENIE_REPORT_EVENT = 0x07
GENIE_REPORT_OBJ = 0x05

# Common widget object IDs
GENIE_OBJ_LED = 0x00
GENIE_OBJ_SLIDER = 0x01
GENIE_OBJ_GAUGE = 0x02
GENIE_OBJ_FORM = 0x0A
GENIE_OBJ_WINBUTTON = 0x06
GENIE_OBJ_STRING = 0x1A


class GenieEvent:
    def __init__(self, cmd, obj, index, value):
        self.cmd = cmd
        self.obj = obj
        self.index = index
        self.value = value


class Genie:

    def __init__(self, uart):
        self.uart = uart
        self.event_handler = None
        self.event_queue = []

    def begin(self):
        time.sleep(0.35)

    def attach_event_handler(self, handler):
        self.event_handler = handler

    def _checksum(self, data):
        return sum(data) & 0xFF

    def _send_frame(self, cmd, obj, index, value):

        frame = bytearray(6)

        frame[0] = cmd
        frame[1] = obj
        frame[2] = index
        frame[3] = (value >> 8) & 0xFF
        frame[4] = value & 0xFF
        frame[5] = self._checksum(frame[:5])

        self.uart.write(frame)

    def write_object(self, obj, index, value):
        self._send_frame(GENIE_WRITE_OBJ, obj, index, value)

    def read_object(self, obj, index):
        self._send_frame(GENIE_READ_OBJ, obj, index, 0)

    def write_string(self, index, text):

        payload = text.encode("ascii")

        header = bytearray(3)
        header[0] = GENIE_WRITE_STR
        header[1] = index
        header[2] = len(payload)

        checksum = (sum(header) + sum(payload)) & 0xFF

        self.uart.write(header)
        self.uart.write(payload)
        self.uart.write(bytes([checksum]))

    def _parse_frame(self, frame):

        cmd = frame[0]
        obj = frame[1]
        index = frame[2]
        value = (frame[3] << 8) | frame[4]

        return GenieEvent(cmd, obj, index, value)

    def do_events(self):

        while self.uart.in_waiting >= 6:

            frame = self.uart.read(6)

            if not frame:
                return

            if self._checksum(frame[:5]) != frame[5]:
                return

            event = self._parse_frame(frame)

            if self.event_handler:
                self.event_handler(event)

            else:
                self.event_queue.append(event)

    def get_event(self):

        if self.event_queue:
            return self.event_queue.pop(0)

        return None

