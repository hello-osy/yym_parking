import os
import select
import struct
import termios
import time


BAUDRATES = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: termios.B230400,
    460800: termios.B460800,
}


class SerialPort:
    """Small POSIX serial wrapper for the standalone LiDAR node."""

    def __init__(self, port, baudrate, timeout=0.02, write_timeout=0.2):
        self.port = port
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.configure(baudrate)

    def configure(self, baudrate):
        attrs = termios.tcgetattr(self.fd)
        speed = BAUDRATES.get(int(baudrate))
        if speed is None:
            raise ValueError(f'unsupported baudrate: {baudrate}')

        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[3] = 0
        attrs[4] = speed
        attrs[5] = speed
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def set_dtr(self, enabled):
        if not hasattr(termios, 'TIOCMGET'):
            return
        bits = struct.unpack('I', fcntl_ioctl(self.fd, termios.TIOCMGET, b'\0' * 4))[0]
        if enabled:
            bits |= termios.TIOCM_DTR
        else:
            bits &= ~termios.TIOCM_DTR
        fcntl_ioctl(self.fd, termios.TIOCMSET, struct.pack('I', bits))

    def read(self, size):
        if size <= 0:
            return b''
        data = bytearray()
        deadline = time.monotonic() + self.timeout
        while len(data) < size:
            remaining = max(0.0, deadline - time.monotonic())
            ready, _, _ = select.select([self.fd], [], [], remaining)
            if not ready:
                break
            try:
                chunk = os.read(self.fd, size - len(data))
            except BlockingIOError:
                continue
            if not chunk:
                break
            data.extend(chunk)
        return bytes(data)

    def readline(self):
        deadline = time.monotonic() + self.timeout
        data = bytearray()
        while time.monotonic() < deadline:
            chunk = self.read(1)
            if not chunk:
                continue
            data.extend(chunk)
            if chunk == b'\n':
                break
        return bytes(data)

    def write(self, data):
        _, writable, _ = select.select([], [self.fd], [], self.write_timeout)
        if not writable:
            raise TimeoutError('serial write timeout')
        return os.write(self.fd, data)

    def flush(self):
        termios.tcdrain(self.fd)

    def reset_input_buffer(self):
        termios.tcflush(self.fd, termios.TCIFLUSH)

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


def fcntl_ioctl(fd, request, argument):
    import fcntl

    return fcntl.ioctl(fd, request, argument)


def open_serial(port, baudrate, timeout=0.02, write_timeout=0.2, dtr=None):
    try:
        import serial

        handle = serial.Serial(
            port,
            baudrate,
            timeout=timeout,
            write_timeout=write_timeout,
        )
        if dtr is not None:
            handle.dtr = dtr
        return handle
    except ImportError:
        handle = SerialPort(port, baudrate, timeout, write_timeout)
        if dtr is not None:
            handle.set_dtr(dtr)
        return handle
