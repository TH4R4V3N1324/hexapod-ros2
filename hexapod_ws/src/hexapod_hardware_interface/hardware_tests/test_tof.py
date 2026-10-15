#!/usr/bin/env python3
"""
Test the Arducam ToF camera (B0410): open it, stream depth frames and report the frame rate,
how much of the image has a confident depth, and the distance at the centre of the image.

Usage: test_tof.py [--index 0] [--range 4000] [--seconds 10] [--save depth.pgm]

Run on the Pi itself, not in the container. It needs Arducam's SDK and its camera overlay,
from https://github.com/ArduCAM/Arducam_tof_camera (Install_dependencies.sh, then
pip install ArducamDepthCamera). Point it at a wall or box a known distance away and
compare the centre distance with a tape measure.
"""

import argparse
import sys
import time

try:
    import ArducamDepthCamera as ac
    import numpy as np
except ImportError as error:
    sys.exit(f'{error.name} is missing: install the Arducam ToF SDK (see test_tof.py --help)')

CONFIDENCE_THRESHOLD = 30  # Arducam's examples treat lower-confidence pixels as invalid
CENTRE_SIZE = 10           # pixels, side of the centre patch whose median distance is shown


def save_pgm(path, depth, max_range):
    """Save a depth frame as an 8-bit greyscale PGM, near = bright, invalid = black."""
    image = np.where(depth > 0, 255 - np.clip(depth / max_range * 255, 0, 255), 0).astype(np.uint8)
    with open(path, 'wb') as file:
        file.write(f'P5 {image.shape[1]} {image.shape[0]} 255\n'.encode())
        file.write(image.tobytes())


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--index', type=int, default=0, help='camera index (which CSI port the overlay set up)')
    parser.add_argument('--range', type=int, default=4000, help='2000 (near mode) or 4000 mm (far mode)')
    parser.add_argument('--seconds', type=float, default=10.0)
    parser.add_argument('--save', help='save the last depth frame to this .pgm file')
    args = parser.parse_args()
    failures = 0

    camera = ac.ArducamCamera()
    result = camera.open(ac.Connection.CSI, args.index)
    if result != 0:
        print(f'  FAIL  Failed to open the ToF camera on index {args.index} (error {result})')
        print('        Check the ribbon cable and that config.txt has the arducam-pivariety overlay for its port,')
        print('        then reboot. "rpicam-hello --list-cameras" should list an arducam-pivariety camera.')
        print('\nResult: FAIL')
        return 1

    try:
        result = camera.start(ac.FrameType.DEPTH)
        if result != 0:
            print(f'  FAIL  Failed to start depth streaming (error {result})')
            print('\nResult: FAIL')
            return 1
        camera.setControl(ac.Control.RANGE, args.range)
        max_range = camera.getControl(ac.Control.RANGE)
        info = camera.getCameraInfo()
        print(f'  PASS  ToF camera open, {info.width}x{info.height}, range {max_range} mm')
        print(f'\nStreaming for {args.seconds:.0f} s, Ctrl+C to stop early.\n')

        frames = 0
        timeouts = 0
        confident_fractions = []
        last_depth = None
        start = time.monotonic()
        last_print = 0.0
        try:
            while time.monotonic() - start < args.seconds:
                frame = camera.requestFrame(2000)
                if frame is None or not isinstance(frame, ac.DepthData):
                    timeouts += 1
                    continue
                depth = np.array(frame.depth_data, dtype=np.float32)
                confidence = np.array(frame.confidence_data, dtype=np.float32)
                camera.releaseFrame(frame)
                frames += 1

                depth[confidence < CONFIDENCE_THRESHOLD] = 0.0
                last_depth = depth
                confident_fractions.append(np.count_nonzero(depth) / depth.size)

                now = time.monotonic()
                if now - last_print >= 0.5:
                    last_print = now
                    rows, cols = depth.shape
                    centre = depth[(rows - CENTRE_SIZE) // 2:(rows + CENTRE_SIZE) // 2,
                                   (cols - CENTRE_SIZE) // 2:(cols + CENTRE_SIZE) // 2]
                    valid = centre[centre > 0]
                    centre_text = f'{np.median(valid) / 1000:.3f} m' if valid.size else 'no confident depth'
                    print(f'centre {centre_text:>20}   confident pixels {100 * confident_fractions[-1]:5.1f}%')
        except KeyboardInterrupt:
            pass
        elapsed = time.monotonic() - start
    finally:
        camera.stop()
        camera.close()

    print(f'\nSummary of {frames} frames:')
    if frames == 0:
        print('  FAIL  No depth frames arrived')
        failures += 1
    else:
        fps = frames / elapsed
        print(f'  {"PASS" if fps >= 10 else "WARN"}  {fps:.1f} frames/s')
        mean_confident = 100 * sum(confident_fractions) / len(confident_fractions)
        print(f'  {"PASS" if mean_confident >= 20 else "WARN"}  {mean_confident:.0f}% of pixels confident on average'
              + ('' if mean_confident >= 20 else ' (nothing within range, or strong sunlight?)'))
    if timeouts:
        print(f'  WARN  {timeouts} frame requests timed out')
    if args.save and last_depth is not None:
        save_pgm(args.save, last_depth, max_range)
        print(f'        Saved the last frame to {args.save} (near = bright, no depth = black)')
    print(f'\nResult: {"FAIL" if failures else "PASS"}')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
