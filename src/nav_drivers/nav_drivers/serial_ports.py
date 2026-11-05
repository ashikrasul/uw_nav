"""Find a serial port by what answers on it, so USB adapters may come up under any name.

Plain Python (no ROS). Probes open ports with exclusive=True: a port another node holds is
reported busy and skipped, so two nodes never talk to the same device.
"""

import glob
import time

import serial


def find_port(pattern, probe):
    """Try each port matching `pattern` (e.g. /dev/ttyUSB*) in sorted order.

    probe(port) returns an open device (anything not None) if it recognises the device, else None.
    Returns (port, device) for the first match, or (None, text listing what was tried and why).
    """
    tried = []
    for port in sorted(glob.glob(pattern)):
        try:
            device = probe(port)
        except (serial.SerialException, OSError) as e:   # busy, no permission, unplugged
            tried.append(f'{port} ({e})')
            continue
        if device is not None:
            return port, device
        tried.append(f'{port} (no answer)')
    return None, ', '.join(tried) or f'no port matches {pattern}'


def nmea_probe(bauds, listen_s):
    """Return a probe that only listens: the open serial.Serial if NMEA ($GP/$GN) arrives."""
    def probe(port):
        uart = serial.Serial(port, bauds[0], timeout=0.1, exclusive=True)
        try:
            for baud in bauds:
                uart.baudrate = baud
                uart.reset_input_buffer()
                end = time.monotonic() + listen_s
                while time.monotonic() < end:
                    line = uart.readline()
                    if b'$GP' in line or b'$GN' in line:
                        return uart
        except BaseException:
            uart.close()
            raise
        uart.close()
        return None
    return probe
