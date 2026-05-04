"""
genie_circuitpython.py
======================
CircuitPython port of the 4D Systems ViSi-Genie Arduino Library
Original: https://github.com/4dsystems/ViSi-Genie-Arduino-Library
License: GPL-3.0

Ported for CircuitPython by Claude (Anthropic), 2026.

Usage example
-------------
import busio
import board
from genie_circuitpython import Genie, GENIE_OBJ_SLIDER, GENIE_OBJ_LED_DIGITS
from genie_circuitpython import GENIE_REPORT_EVENT

uart = busio.UART(board.TX, board.RX, baudrate=115200)
genie = Genie()
genie.begin(uart)

def my_event_handler(event):
    if genie.event_is(event, GENIE_REPORT_EVENT, GENIE_OBJ_SLIDER, 0):
        val = genie.get_event_data(event)
        genie.write_object(GENIE_OBJ_LED_DIGITS, 0, val)

genie.attach_event_handler(my_event_handler)

while True:
    genie.do_events()
"""

import struct
import time

# ---------------------------------------------------------------------------
# Command bytes
# ---------------------------------------------------------------------------
GENIE_ACK               = 0x06
GENIE_NAK               = 0x15

GENIE_READ_OBJ          = 0x00
GENIE_WRITE_OBJ         = 0x01
GENIE_WRITE_STR         = 0x02
GENIE_WRITE_STRU        = 0x03
GENIE_WRITE_CONTRAST    = 0x04
GENIE_REPORT_OBJ        = 0x05
GENIE_REPORT_EVENT      = 0x06
GENIE_WRITE_MAGIC_BYTES = 0x0F
GENIE_WRITE_MAGIC_DBYTES= 0x10

# Ping / connection states (used internally in event queue)
GENIE_PING              = 0xFF
GENIE_READY             = 0x00
GENIE_DISCONNECTED      = 0x01

# Magic report bytes
GENIEM_REPORT_BYTES     = 0x07
GENIEM_REPORT_DBYTES    = 0x08
GENIEM_WRITE_BYTES      = 0x09
GENIEM_WRITE_DBYTES     = 0x0A

# Internal LedDigits format helpers
GENIE_INT16             = 0x00
GENIE_FLOAT             = 0x01
GENIE_INT32             = 0x02

# ---------------------------------------------------------------------------
# Object / Widget constants
# ---------------------------------------------------------------------------
GENIE_OBJ_DIPSW                 = 0
GENIE_OBJ_KNOB                  = 1
GENIE_OBJ_ROCKERSW              = 2
GENIE_OBJ_ROTARYSW              = 3
GENIE_OBJ_SLIDER                = 4
GENIE_OBJ_TRACKBAR              = 5
GENIE_OBJ_WINBUTTON             = 6
GENIE_OBJ_ANGULAR_METER         = 7
GENIE_OBJ_COOL_GAUGE            = 8
GENIE_OBJ_CUSTOM_DIGITS         = 9
GENIE_OBJ_FORM                  = 10
GENIE_OBJ_GAUGE                 = 11
GENIE_OBJ_IMAGE                 = 12
GENIE_OBJ_KEYBOARD              = 13
GENIE_OBJ_LED                   = 14
GENIE_OBJ_LED_DIGITS            = 15
GENIE_OBJ_METER                 = 16
GENIE_OBJ_STRINGS               = 17
GENIE_OBJ_THERMOMETER           = 18
GENIE_OBJ_USER_LED              = 19
GENIE_OBJ_VIDEO                 = 20
GENIE_OBJ_STATIC_TEXT           = 21
GENIE_OBJ_SOUND                 = 22
GENIE_OBJ_TIMER                 = 23
GENIE_OBJ_SPECTRUM              = 24
GENIE_OBJ_SCOPE                 = 25
GENIE_OBJ_TANK                  = 26
GENIE_OBJ_USERIMAGES            = 27
GENIE_OBJ_PINOUTPUT             = 28
GENIE_OBJ_PININPUT              = 29
GENIE_OBJ_4DBUTTON              = 30
GENIE_OBJ_ANIBUTTON             = 31
GENIE_OBJ_COLORPICKER           = 32
GENIE_OBJ_USERBUTTON            = 33
GENIE_OBJ_SMARTGAUGE            = 34
GENIE_OBJ_SMARTSLIDER           = 35
GENIE_OBJ_SMARTKNOB             = 36
# Aliases kept for backward compatibility
GENIE_OBJ_ISMARTGAUGE           = 34
GENIE_OBJ_ISMARTSLIDER          = 35
GENIE_OBJ_ISMARTKNOB            = 36

# Internal / Inherent widgets
GENIE_OBJ_ILED_DIGITS_H         = 37
GENIE_OBJ_ILED_DIGITS_L         = 38
GENIE_OBJ_IANGULAR_METER        = 39
GENIE_OBJ_IGAUGE                = 40
GENIE_OBJ_ILED                  = 41
GENIE_OBJ_INEEDLE               = 42
GENIE_OBJ_IRULER                = 43
GENIE_OBJ_ILED_DIGIT            = 44
GENIE_OBJ_ILED_DIGITS           = 45
GENIE_OBJ_IBUTTOND              = 46
GENIE_OBJ_IDIAL                 = 47
GENIE_OBJ_ISWITCH               = 48
GENIE_OBJ_ISLIDERE              = 49
GENIE_OBJ_IBUTTONE              = 50
GENIE_OBJ_ITOGGLE_INPUT         = 51
GENIE_OBJ_ILABELB               = 52
GENIE_OBJ_IUSER_GAUGE           = 53
GENIE_OBJ_IMEDIA_BUTTON         = 54
GENIE_OBJ_IMEDIA_GAUGE          = 55
GENIE_OBJ_IMEDIA_THERMOMETER    = 56
GENIE_OBJ_IMEDIA_ROTARY         = 57
GENIE_OBJ_IMEDIA_LED            = 58
GENIE_OBJ_IMEDIA_SLIDER         = 59
GENIE_OBJ_IROTARY_INPUT         = 60
GENIE_OBJ_ISWITCHB              = 61
GENIE_OBJ_ISLIDERH              = 62
GENIE_OBJ_ISLIDERG              = 63
GENIE_OBJ_ISLIDERF              = 64
GENIE_OBJ_ISLIDERD              = 65
GENIE_OBJ_ISLIDERC              = 66
GENIE_OBJ_ILINEAR_INPUT         = 67

# ---------------------------------------------------------------------------
# Frame structure helper (replaces C struct genieFrame)
# ---------------------------------------------------------------------------
class GenieFrame:
    """
    Mirrors the genieFrame structure used by the Arduino library.

    Attributes
    ----------
    cmd : int
        The command byte (e.g. GENIE_REPORT_EVENT, GENIE_REPORT_OBJ).
    object : int
        The widget type (e.g. GENIE_OBJ_SLIDER).
    index : int
        The index of the widget on the display form.
    data_hi : int
        High byte of the 16-bit value payload.
    data_lo : int
        Low byte of the 16-bit value payload.
    """
    __slots__ = ("cmd", "object", "index", "data_hi", "data_lo")

    def __init__(self):
        self.cmd      = 0
        self.object   = 0
        self.index    = 0
        self.data_hi  = 0
        self.data_lo  = 0

    @property
    def data(self):
        """Return the 16-bit combined data value."""
        return (self.data_hi << 8) | self.data_lo

    def __repr__(self):
        return (
            f"GenieFrame(cmd=0x{self.cmd:02X}, obj=0x{self.object:02X}, "
            f"idx={self.index}, data=0x{self.data:04X})"
        )


# ---------------------------------------------------------------------------
# Main Genie class
# ---------------------------------------------------------------------------
class Genie:
    """
    CircuitPython driver for 4D Systems displays running ViSi-Genie firmware.

    Parameters
    ----------
    queue_size : int
        Maximum number of frames that can be queued before older ones are
        dropped.  Default is 16, matching the Arduino library.

    Notes
    -----
    CircuitPython does not have hardware interrupts available to user code, so
    reception is polled inside ``do_events()``.  Call ``do_events()`` as often
    as possible inside your main loop for the best responsiveness.

    Unlike the Arduino library there is no separate ``Stream`` class.  Pass
    a ``busio.UART`` (or any object with ``read()`` / ``write()`` methods) to
    ``begin()``.
    """

    GENIE_QUEUE_SIZE = 16

    def __init__(self, queue_size: int = 16):
        self._uart = None
        self._event_handler = None
        self._magic_byte_handler = None
        self._magic_dbyte_handler = None
        self._queue = []
        self._queue_size = queue_size
        self._rx_buf = bytearray()
        self._display_detected = False
        # Internal byte buffer used by GetNextByte / GetNextDoubleByte
        self._magic_buf = bytearray()
        self._magic_ptr = 0

    # ------------------------------------------------------------------
    # Initialisation
    # ------------------------------------------------------------------
    def begin(self, uart) -> None:
        """
        Assign the UART/serial stream and initialise the driver.

        Parameters
        ----------
        uart : busio.UART or compatible
            A serial object that exposes ``read(n)`` and ``write(data)``
            methods and has a ``baudrate`` attribute.  Create it with
            ``busio.UART(TX, RX, baudrate=115200)`` before calling this.
        """
        self._uart = uart
        self._rx_buf = bytearray()
        self._queue.clear()
        self._display_detected = False

    # ------------------------------------------------------------------
    # Event handler registration
    # ------------------------------------------------------------------
    def attach_event_handler(self, handler) -> None:
        """
        Register a callable to be invoked when the display sends a message.

        The callable receives no arguments; call ``dequeue_event()`` inside
        it to retrieve the pending ``GenieFrame``.

        Parameters
        ----------
        handler : callable
            A zero-argument function, e.g. ``def my_handler(): ...``
        """
        self._event_handler = handler

    def attach_magic_byte_reader(self, handler) -> None:
        """
        Register a handler for Magic Byte events.

        The handler signature must be ``handler(index: int, length: int)``.
        Inside the handler call ``get_next_byte()`` *length* times to consume
        the incoming bytes.
        """
        self._magic_byte_handler = handler

    def attach_magic_double_byte_reader(self, handler) -> None:
        """
        Register a handler for Magic Double-Byte events.

        The handler signature must be ``handler(index: int, length: int)``.
        Inside the handler call ``get_next_double_byte()`` *length* times.
        """
        self._magic_dbyte_handler = handler

    # ------------------------------------------------------------------
    # Helpers used inside magic handlers
    # ------------------------------------------------------------------
    def get_next_byte(self) -> int:
        """
        Read the next byte from the internal magic receive buffer.

        Must only be called from inside a magic byte handler registered with
        ``attach_magic_byte_reader()``.
        """
        if self._magic_ptr < len(self._magic_buf):
            b = self._magic_buf[self._magic_ptr]
            self._magic_ptr += 1
            return b
        return 0

    def get_next_double_byte(self) -> int:
        """
        Read the next 16-bit value (big-endian) from the internal magic buffer.

        Must only be called from inside a handler registered with
        ``attach_magic_double_byte_reader()``.
        """
        if self._magic_ptr + 1 < len(self._magic_buf):
            hi = self._magic_buf[self._magic_ptr]
            lo = self._magic_buf[self._magic_ptr + 1]
            self._magic_ptr += 2
            return (hi << 8) | lo
        return 0

    # ------------------------------------------------------------------
    # Queue management
    # ------------------------------------------------------------------
    def _enqueue(self, frame: GenieFrame) -> None:
        if len(self._queue) >= self._queue_size:
            self._queue.pop(0)   # drop oldest
        self._queue.append(frame)
        if self._event_handler is not None:
            self._event_handler()

    def dequeue_event(self) -> GenieFrame:
        """
        Remove and return the oldest ``GenieFrame`` from the receive queue.

        Returns
        -------
        GenieFrame
            The oldest pending event, or an empty ``GenieFrame`` if the queue
            is empty.
        """
        if self._queue:
            return self._queue.pop(0)
        return GenieFrame()

    # ------------------------------------------------------------------
    # CRC / checksum
    # ------------------------------------------------------------------
    @staticmethod
    def _calc_checksum(data: bytes) -> int:
        """XOR checksum used by the Genie protocol."""
        crc = 0
        for b in data:
            crc ^= b
        return crc

    # ------------------------------------------------------------------
    # Low-level transmit
    # ------------------------------------------------------------------
    def _send(self, packet: bytearray) -> bool:
        """
        Append checksum to *packet* and transmit over UART.

        Returns True if the display acknowledges with ACK, False on NAK/timeout.
        """
        if self._uart is None:
            return False
        checksum = self._calc_checksum(packet)
        packet.append(checksum)
        self._uart.write(bytes(packet))
        return self._wait_for_ack()

    def _wait_for_ack(self, timeout_ms: int = 1000) -> bool:
        """
        Block until ACK or NAK is received, or timeout expires.

        Returns True on ACK, False otherwise.
        """
        deadline = time.monotonic() + timeout_ms / 1000.0
        while time.monotonic() < deadline:
            b = self._uart.read(1)
            if b:
                if b[0] == GENIE_ACK:
                    return True
                if b[0] == GENIE_NAK:
                    return False
        return False

    # ------------------------------------------------------------------
    # Public write commands
    # ------------------------------------------------------------------
    def write_contrast(self, value: int) -> bool:
        """
        Set the display brightness / contrast.

        Parameters
        ----------
        value : int
            Brightness level.  Typically 0–15 (0 = off, 15 = maximum).
            Some displays only support 0 (off) / 1 (on).

        Returns
        -------
        bool
            True if the display acknowledged the command.
        """
        value = max(0, min(15, int(value)))
        pkt = bytearray([GENIE_WRITE_CONTRAST, value])
        return self._send(pkt)

    def write_object(self, obj: int, index: int, data: int) -> bool:
        """
        Write a 16-bit value to a widget on the display.

        Parameters
        ----------
        obj : int
            Widget type constant (e.g. ``GENIE_OBJ_GAUGE``).
        index : int
            Zero-based index of the widget on the current form.
        data : int
            New value for the widget (0–65535).

        Returns
        -------
        bool
            True if the display acknowledged the command.
        """
        data = int(data) & 0xFFFF
        pkt = bytearray([
            GENIE_WRITE_OBJ,
            obj & 0xFF,
            index & 0xFF,
            (data >> 8) & 0xFF,
            data & 0xFF,
        ])
        return self._send(pkt)

    def read_object(self, obj: int, index: int) -> bool:
        """
        Request a value report from a widget on the display.

        The display will respond asynchronously with a ``GENIE_REPORT_OBJ``
        message that arrives via ``do_events()`` and the registered event
        handler.

        Parameters
        ----------
        obj : int
            Widget type constant.
        index : int
            Zero-based widget index.

        Returns
        -------
        bool
            True if the display acknowledged the read request.
        """
        pkt = bytearray([GENIE_READ_OBJ, obj & 0xFF, index & 0xFF])
        return self._send(pkt)

    # ------------------------------------------------------------------
    # Internal LedDigits helpers
    # ------------------------------------------------------------------
    def write_int_led_digits_int16(self, index: int, data: int) -> bool:
        """
        Write a signed 16-bit integer to an Internal LedDigits widget.

        The widget *Format* in Workshop4 must be set to **Int16**.
        Available on Diablo and Pixxi displays only.

        Parameters
        ----------
        index : int
            Index of the Internal LedDigits widget.
        data : int
            Signed 16-bit integer value (−32768 to 32767).
        """
        data = int(data) & 0xFFFF
        return self.write_object(GENIE_OBJ_ILED_DIGITS, index, data)

    def write_int_led_digits_int32(self, index: int, data: int) -> bool:
        """
        Write a signed 32-bit integer to an Internal LedDigits widget.

        The widget *Format* in Workshop4 must be set to **Int32** (or Long).
        Splits the value across the high and low byte objects.

        Parameters
        ----------
        index : int
            Index of the Internal LedDigits widget.
        data : int
            Signed 32-bit integer value.
        """
        data = int(data) & 0xFFFFFFFF
        hi = (data >> 16) & 0xFFFF
        lo = data & 0xFFFF
        ok = self.write_object(GENIE_OBJ_ILED_DIGITS_H, index, hi)
        ok = self.write_object(GENIE_OBJ_ILED_DIGITS_L, index, lo) and ok
        return ok

    def write_int_led_digits_float(self, index: int, data: float) -> bool:
        """
        Write a 32-bit IEEE 754 float to an Internal LedDigits widget.

        The widget *Format* in Workshop4 must be set to a **Float** option.

        Parameters
        ----------
        index : int
            Index of the Internal LedDigits widget.
        data : float
            Float value to display.
        """
        raw = struct.pack(">f", float(data))
        hi = struct.unpack(">H", raw[0:2])[0]
        lo = struct.unpack(">H", raw[2:4])[0]
        ok = self.write_object(GENIE_OBJ_ILED_DIGITS_H, index, hi)
        ok = self.write_object(GENIE_OBJ_ILED_DIGITS_L, index, lo) and ok
        return ok

    # Convenience alias matching Arduino overload naming
    def write_int_led_digits(self, index: int, data) -> bool:
        """
        Dispatch to the correct Internal LedDigits writer based on type.

        Accepts ``int`` (written as Int16 if –32768 ≤ value ≤ 65535,
        otherwise as Int32) or ``float``.
        """
        if isinstance(data, float):
            return self.write_int_led_digits_float(index, data)
        data = int(data)
        if -32768 <= data <= 65535:
            return self.write_int_led_digits_int16(index, data)
        return self.write_int_led_digits_int32(index, data)

    # ------------------------------------------------------------------
    # String writing
    # ------------------------------------------------------------------
    def write_str(self, index: int, text, base: int = 10) -> bool:
        """
        Write a string or numeric value to a String widget.

        Parameters
        ----------
        index : int
            Index of the target String widget.
        text : str | int | float
            Value to display.  Integers are converted using *base*; floats
            are formatted with two decimal places.
        base : int
            Number base for integer formatting (default 10).  Pass 16 for
            hex, 8 for octal, 2 for binary.

        Returns
        -------
        bool
            True if the display acknowledged the command.
        """
        if isinstance(text, float):
            s = f"{text:.2f}"
        elif isinstance(text, int):
            if base == 16:
                s = hex(text)[2:].upper()
            elif base == 8:
                s = oct(text)[2:]
            elif base == 2:
                s = bin(text)[2:]
            else:
                s = str(text)
        else:
            s = str(text)

        encoded = s.encode("ascii", errors="replace") + b"\x00"
        pkt = bytearray([GENIE_WRITE_STR, index & 0xFF, len(encoded)])
        pkt.extend(encoded)
        return self._send(pkt)

    def write_str_u(self, index: int, ustring) -> bool:
        """
        Write a Unicode (16-bit) string to a String widget.

        Parameters
        ----------
        index : int
            Index of the target String widget.
        ustring : list[int] or bytes-like of uint16 values (big-endian).
            Each element is a 16-bit Unicode code point.
            The list is automatically null-terminated.

        Returns
        -------
        bool
            True if the display acknowledged the command.
        """
        words = list(ustring)
        if not words or words[-1] != 0:
            words.append(0)
        byte_len = len(words) * 2
        pkt = bytearray([GENIE_WRITE_STRU, index & 0xFF, byte_len])
        for w in words:
            pkt.append((w >> 8) & 0xFF)
            pkt.append(w & 0xFF)
        return self._send(pkt)

    # ------------------------------------------------------------------
    # Inherent Label writing
    # ------------------------------------------------------------------
    def write_inh_label(self, index: int, text=None, base: int = 10) -> bool:
        """
        Write text to an Inherent Label (ILabelB) widget.

        When called without *text* the display resets the label to its
        default Workshop4 content.  Otherwise behaves identically to
        ``write_str()``.

        Parameters
        ----------
        index : int
            Index of the Inherent Label widget.
        text : str | int | float | None
            Text to display, or ``None`` to restore the default label text.
        base : int
            Number base for integer formatting.

        Returns
        -------
        bool
            True if the display acknowledged the command.
        """
        if text is None:
            # Reset label to default — send an empty string
            pkt = bytearray([GENIE_WRITE_STR, GENIE_OBJ_ILABELB, index & 0xFF, 0x01, 0x00])
            return self._send(pkt)
        return self.write_str(index, text, base)

    # ------------------------------------------------------------------
    # Magic bytes
    # ------------------------------------------------------------------
    def write_magic_bytes(self, index: int, data: bytes) -> bool:
        """
        Send a sequence of magic bytes to a MagicObject on the display.

        Parameters
        ----------
        index : int
            Index of the target MagicObject.
        data : bytes or bytearray
            The raw bytes to send.

        Returns
        -------
        bool
            True if the display acknowledged the command.
        """
        length = len(data)
        pkt = bytearray([GENIEM_WRITE_BYTES, index & 0xFF, length & 0xFF])
        pkt.extend(data)
        return self._send(pkt)

    def write_magic_dbytes(self, index: int, data) -> bool:
        """
        Send a sequence of 16-bit magic double-bytes to a MagicObject.

        Parameters
        ----------
        index : int
            Index of the target MagicObject.
        data : list[int]
            List of 16-bit unsigned integers to send.

        Returns
        -------
        bool
            True if the display acknowledged the command.
        """
        length = len(data)
        pkt = bytearray([GENIEM_WRITE_DBYTES, index & 0xFF, length & 0xFF])
        for w in data:
            pkt.append((w >> 8) & 0xFF)
            pkt.append(w & 0xFF)
        return self._send(pkt)

    # ------------------------------------------------------------------
    # Event helpers
    # ------------------------------------------------------------------
    def event_is(self, frame: GenieFrame, cmd: int, obj: int, index: int) -> bool:
        """
        Check whether *frame* matches the given command, object and index.

        Parameters
        ----------
        frame : GenieFrame
            The frame to test.
        cmd : int
            Expected command byte (e.g. ``GENIE_REPORT_EVENT``).
        obj : int
            Expected widget type.
        index : int
            Expected widget index.

        Returns
        -------
        bool
            True if all three fields match.
        """
        return (frame.cmd == cmd and frame.object == obj and frame.index == index)

    def get_event_data(self, frame: GenieFrame) -> int:
        """
        Extract the 16-bit value from a ``GenieFrame``.

        Parameters
        ----------
        frame : GenieFrame
            A frame previously retrieved with ``dequeue_event()``.

        Returns
        -------
        int
            The unsigned 16-bit payload value.
        """
        return frame.data

    # ------------------------------------------------------------------
    # Main receive / dispatch loop
    # ------------------------------------------------------------------
    def do_events(self) -> None:
        """
        Process all pending incoming bytes from the display.

        Call this as often as possible inside your main ``while True:`` loop.
        It reads available bytes from the UART, assembles complete Genie
        frames, and invokes the registered event handler for each one.

        Example
        -------
        ::

            while True:
                genie.do_events()
                # ... your code here
        """
        if self._uart is None:
            return

        # Drain whatever is available
        incoming = self._uart.read(64)
        if incoming:
            self._rx_buf.extend(incoming)

        # Parse complete frames out of the buffer
        while len(self._rx_buf) >= 6:
            first = self._rx_buf[0]

            # -----------------------------------------------------------
            # Standard 6-byte frames: cmd + obj + idx + hi + lo + crc
            # -----------------------------------------------------------
            if first in (GENIE_REPORT_OBJ, GENIE_REPORT_EVENT):
                if len(self._rx_buf) < 6:
                    break
                frame_bytes = self._rx_buf[:6]
                expected_crc = self._calc_checksum(frame_bytes[:5])
                if frame_bytes[5] != expected_crc:
                    # Bad checksum — discard one byte and retry
                    self._rx_buf = self._rx_buf[1:]
                    continue
                frame = GenieFrame()
                frame.cmd      = frame_bytes[0]
                frame.object   = frame_bytes[1]
                frame.index    = frame_bytes[2]
                frame.data_hi  = frame_bytes[3]
                frame.data_lo  = frame_bytes[4]
                self._rx_buf = self._rx_buf[6:]
                self._display_detected = True
                self._enqueue(frame)

            # -----------------------------------------------------------
            # Magic byte report: cmd(0x07) + idx + len + [data] + crc
            # -----------------------------------------------------------
            elif first == GENIEM_REPORT_BYTES:
                if len(self._rx_buf) < 3:
                    break
                magic_len = self._rx_buf[2]
                total = 3 + magic_len + 1  # header + data + crc
                if len(self._rx_buf) < total:
                    break
                frame_bytes = self._rx_buf[:total]
                expected_crc = self._calc_checksum(frame_bytes[:-1])
                if frame_bytes[-1] != expected_crc:
                    self._rx_buf = self._rx_buf[1:]
                    continue
                idx = self._rx_buf[1]
                payload = bytes(self._rx_buf[3:3 + magic_len])
                self._rx_buf = self._rx_buf[total:]
                if self._magic_byte_handler is not None:
                    self._magic_buf = bytearray(payload)
                    self._magic_ptr = 0
                    self._magic_byte_handler(idx, magic_len)

            # -----------------------------------------------------------
            # Magic double-byte report: 0x08 + idx + len + [data] + crc
            # -----------------------------------------------------------
            elif first == GENIEM_REPORT_DBYTES:
                if len(self._rx_buf) < 3:
                    break
                magic_len = self._rx_buf[2]
                total = 3 + (magic_len * 2) + 1
                if len(self._rx_buf) < total:
                    break
                frame_bytes = self._rx_buf[:total]
                expected_crc = self._calc_checksum(frame_bytes[:-1])
                if frame_bytes[-1] != expected_crc:
                    self._rx_buf = self._rx_buf[1:]
                    continue
                idx = self._rx_buf[1]
                payload = bytes(self._rx_buf[3:3 + magic_len * 2])
                self._rx_buf = self._rx_buf[total:]
                if self._magic_dbyte_handler is not None:
                    self._magic_buf = bytearray(payload)
                    self._magic_ptr = 0
                    self._magic_dbyte_handler(idx, magic_len)

            else:
                # Unrecognised byte — discard and keep scanning
                self._rx_buf = self._rx_buf[1:]
