#!/usr/bin/env python3
"""
Test the RPLIDAR A2M12 directly over its serial port, without rplidar_ros.

Usage: test_lidar [--port /dev/rplidar] [--baud 256000] [--seconds 5]

Reads the lidar's model, firmware and health, spins the motor, scans for a few seconds and
reports the rotation rate, sample rate and nearest object in each 45° sector. Hold a hand in
front of the lidar to check which way 0° faces (the lidar's 0° is its cable side's opposite,
and angles increase clockwise seen from above; rplidar_ros converts to the ROS convention).
"""

import argparse
import math
import struct
import sys
import time

try:
    import serial
except ImportError:
    sys.exit('pyserial is missing: sudo apt install python3-serial')

SYNC = 0xA5
RESPONSE_SYNC = 0x5A
CMD_STOP = 0x25
CMD_RESET = 0x40
CMD_SCAN = 0x20
CMD_GET_INFO = 0x50
CMD_GET_HEALTH = 0x52
CMD_SET_MOTOR_PWM = 0xF0
DEFAULT_MOTOR_PWM = 660
HEALTH = {0: 'good', 1: 'warning', 2: 'error'}


class Report:
    """Counts PASS/WARN/FAIL results and prints each one."""

    def __init__(self):
        self.warnings = 0
        self.failures = 0

    def passed(self, text):
        print(f'  PASS  {text}')

    def warn(self, text):
        self.warnings += 1
        print(f'  WARN  {text}')

    def fail(self, text):
        self.failures += 1
        print(f'  FAIL  {text}')

    def info(self, text):
        print(f'        {text}')

    def finish(self):
        """Print the overall result and return the process exit code."""
        result = 'FAIL' if self.failures else ('PASS with warnings' if self.warnings else 'PASS')
        print(f'\nResult: {result} ({self.warnings} warnings, {self.failures} failures)')
        return 1 if self.failures else 0


def send(port, command, payload=b''):
    """Send a request, with a checksum if it has a payload."""
    packet = bytes([SYNC, command])
    if payload:
        packet += bytes([len(payload)]) + payload
        checksum = 0
        for byte in packet:
            checksum ^= byte
        packet += bytes([checksum])
    port.write(packet)


def read_descriptor(port, timeout=1.0):
    """Wait for a response descriptor and return its data length, or None on timeout."""
    deadline = time.monotonic() + timeout
    previous = None
    while time.monotonic() < deadline:
        byte = port.read(1)
        if not byte:
            continue
        if previous == SYNC and byte[0] == RESPONSE_SYNC:
            rest = port.read(5)
            if len(rest) < 5:
                return None
            return struct.unpack('<I', rest[:4])[0] & 0x3FFFFFFF
        previous = byte[0]
    return None


def request(port, command, timeout=1.0):
    """Send a request and return its response data, or None if the lidar didn't reply."""
    port.reset_input_buffer()
    send(port, command)
    length = read_descriptor(port, timeout)
    if length is None:
        return None
    data = port.read(length)
    return data if len(data) == length else None


def set_motor(port, on):
    """Start or stop the motor. Older A2s switch it with DTR, newer ones take a PWM command."""
    try:
        port.dtr = not on
    except OSError:
        pass  # no DTR line on this adapter, so it must be a PWM model
    send(port, CMD_SET_MOTOR_PWM, struct.pack('<H', DEFAULT_MOTOR_PWM if on else 0))


def scan(port, seconds):
    """Run a standard scan and return the samples as (angle °, distance m, quality, new rotation)."""
    samples = []
    port.reset_input_buffer()
    send(port, CMD_SCAN)
    if read_descriptor(port, 2.0) is None:
        return None
    deadline = time.monotonic() + seconds
    buffer = b''
    while time.monotonic() < deadline:
        buffer += port.read(max(5, port.in_waiting))
        while len(buffer) >= 5:
            b0, b1, b2, b3, b4 = buffer[:5]
            # Each node: start bit and its inverse, then a check bit that is always 1
            start = b0 & 0x01
            if start == ((b0 >> 1) & 0x01) or not (b1 & 0x01):
                buffer = buffer[1:]  # out of step, resynchronise
                continue
            buffer = buffer[5:]
            angle = ((b1 >> 1) | (b2 << 7)) / 64.0
            distance = (b3 | (b4 << 8)) / 4000.0
            samples.append((angle, distance, b0 >> 2, bool(start)))
    send(port, CMD_STOP)
    return samples


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', default='/dev/rplidar')
    parser.add_argument('--baud', type=int, default=256000, help='256000 for the A2M12, 115200 for the A2M8')
    parser.add_argument('--seconds', type=float, default=5.0)
    args = parser.parse_args()
    report = Report()

    try:
        port = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as error:
        report.fail(f'Failed to open {args.port}: {error}')
        report.info('Is it plugged in? /dev/rplidar comes from udev/99-rplidar.rules (see pi/setup_pi.sh).')
        report.info('Is the robot running with the lidar driver? It holds the port; stop it first.')
        return report.finish()

    try:
        send(port, CMD_STOP)
        time.sleep(0.05)

        info = request(port, CMD_GET_INFO)
        if info is None or len(info) < 20:
            report.fail(f'No reply to GET_INFO at {args.baud} baud')
            report.info('Check the baud rate (A2M12 256000, A2M8 115200) and the USB adapter\'s switch/cable.')
            return report.finish()
        model, firmware_minor, firmware_major, hardware = info[0], info[1], info[2], info[3]
        serial_number = info[4:20].hex().upper()
        report.passed(f'RPLIDAR model 0x{model:02X}, firmware {firmware_major}.{firmware_minor:02d}, '
                      f'hardware {hardware}, serial {serial_number}')

        health = request(port, CMD_GET_HEALTH)
        if health is None or len(health) < 3:
            report.warn('No reply to GET_HEALTH')
        else:
            status, error_code = health[0], health[1] | (health[2] << 8)
            text = f'Health {HEALTH.get(status, status)} (error code 0x{error_code:04X})'
            if status == 0:
                report.passed(text)
            else:
                report.fail(text)
                report.info('Power cycle the lidar. A persistent error usually means a motor or laser fault.')

        print(f'\nSpinning up and scanning for {args.seconds:.0f} s...')
        set_motor(port, True)
        time.sleep(2.0)
        samples = scan(port, args.seconds)
        set_motor(port, False)
    finally:
        port.close()

    print()
    if not samples:
        report.fail('No scan data: is the motor spinning? The A2 needs 5 V at up to 1.5 A while spinning up.')
        return report.finish()

    rotations = sum(1 for sample in samples if sample[3])
    rotation_hz = rotations / args.seconds
    valid = [sample for sample in samples if sample[1] > 0.0]
    report.info(f'{len(samples)} samples in {rotations} rotations, {len(valid)} with a range')
    if 5.0 <= rotation_hz <= 15.0:
        report.passed(f'Rotating at {rotation_hz:.1f} Hz')
    else:
        report.warn(f'Rotating at {rotation_hz:.1f} Hz, expected about 10 Hz')
    sample_rate = len(samples) / args.seconds
    report.info(f'{sample_rate:.0f} samples/s (a standard scan; rplidar_ros uses a faster mode)')
    if len(valid) < 0.3 * len(samples):
        report.warn('Under 30% of samples have a range: is the lidar covered, or is the room very large?')
    else:
        report.passed(f'{100 * len(valid) / len(samples):.0f}% of samples have a range')

    if valid:
        distances = [sample[1] for sample in valid]
        report.info(f'Ranges {min(distances):.2f}-{max(distances):.2f} m')
        print('\n  Nearest object per sector (lidar angles, clockwise from 0° seen from above):')
        for sector in range(8):
            low = sector * 45.0
            in_sector = [sample[1] for sample in valid if low <= sample[0] < low + 45.0]
            nearest = f'{min(in_sector):.2f} m' if in_sector else 'nothing in range'
            print(f'    {low:5.0f}°-{low + 45:3.0f}°  {nearest}')
    return report.finish()


if __name__ == '__main__':
    sys.exit(main())
