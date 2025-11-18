#!/usr/bin/env python3
"""
Simple Thruster Verification Test

Sequentially activates each of the 8 thrusters for 1 second.
Used to verify all thrusters are responding before hardware testing.

Test procedure:
- Thrusters activate in order: 1, 2, 3, 4, 5, 6, 7, 8
- Each thruster fires for 1 second
- Confirms all 8 thrusters respond to commands

Usage:
    python3 thruster_test.py

Safety:
- Requires serial connection to microcontroller
- Satellite must be in safe position on test floor
- Keep hands clear during testing
- Ready to cut air supply if needed
"""

import sys
import time
import serial
import logging
from pathlib import Path

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class ThrusterSequentialTest:
    """Sequential thruster verification system."""

    def __init__(self, port: str = '/dev/ttyUSB0', baud: int = 115200):
        """
        Initialize thruster test.

        Args:
            port: Serial port (default: /dev/ttyUSB0 for Linux)
            baud: Baud rate (default: 115200)
        """
        self.port = port
        self.baud = baud
        self.ser = None
        self.test_duration = 1.0  # seconds per thruster

    def connect(self) -> bool:
        """
        Establish serial connection to microcontroller.

        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(0.5)  # Wait for connection to stabilize
            logger.info(f"✓ Connected to {self.port} @ {self.baud} baud")
            return True
        except serial.SerialException as e:
            logger.error(f"✗ Failed to connect to {self.port}: {e}")
            logger.error("Check:")
            logger.error("  - USB cable is connected")
            logger.error("  - Port number is correct (try: ls /dev/tty*)")
            logger.error("  - Microcontroller is powered")
            return False

    def disconnect(self):
        """Close serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info("Disconnected from microcontroller")

    def fire_thruster(self, thruster_num: int) -> bool:
        """
        Fire a single thruster by number (1-8).

        Args:
            thruster_num: Thruster number (1-8)

        Returns:
            True if command sent successfully, False otherwise
        """
        if not isinstance(self.ser, serial.Serial) or not self.ser.is_open:
            logger.error("Serial connection not open")
            return False

        if thruster_num < 1 or thruster_num > 8:
            logger.error(f"Invalid thruster number: {thruster_num}")
            return False

        try:
            # Convert thruster number (1-8) to bitmask (bit 0-7)
            bitmask = 1 << (thruster_num - 1)

            # Send command: byte 1 = 0x01 (thruster command), byte 2 = bitmask
            command = bytes([0x01, bitmask])
            self.ser.write(command)
            self.ser.flush()

            logger.info(f"Fired thruster {thruster_num} (bitmask: 0x{bitmask:02x})")
            return True

        except serial.SerialException as e:
            logger.error(f"Failed to fire thruster {thruster_num}: {e}")
            return False

    def stop_all(self) -> bool:
        """
        Stop all thrusters by sending zero command.

        Returns:
            True if command sent successfully, False otherwise
        """
        if not isinstance(self.ser, serial.Serial) or not self.ser.is_open:
            logger.error("Serial connection not open")
            return False

        try:
            # Send zero bitmask to stop all thrusters
            command = bytes([0x01, 0x00])
            self.ser.write(command)
            self.ser.flush()
            logger.info("Stopped all thrusters")
            return True
        except serial.SerialException as e:
            logger.error(f"Failed to stop thrusters: {e}")
            return False

    def run_sequential_test(self) -> bool:
        """
        Run sequential thruster test.

        Fires each thruster 1-8 in sequence for 1 second each.

        Returns:
            True if all thrusters tested successfully, False on error
        """
        logger.info("=" * 60)
        logger.info("SEQUENTIAL THRUSTER TEST")
        logger.info("=" * 60)
        logger.info(f"Testing all 8 thrusters ({self.test_duration}s each)")
        logger.info("Press Ctrl+C to stop immediately")
        logger.info("=" * 60)

        all_success = True

        try:
            for thruster_num in range(1, 9):
                # Fire thruster
                logger.info(f"\n[{thruster_num}/8] Testing thruster {thruster_num}...")
                if not self.fire_thruster(thruster_num):
                    logger.error(f"Failed to fire thruster {thruster_num}")
                    all_success = False
                    continue

                # Wait for duration
                for remaining in range(int(self.test_duration), 0, -1):
                    print(f"  Firing... {remaining}s remaining", end='\r')
                    time.sleep(1)
                print("  Firing... Done!            ")

                # Stop thruster
                if not self.stop_all():
                    logger.error(f"Failed to stop after thruster {thruster_num}")
                    all_success = False

                # Brief pause between thrusters
                time.sleep(0.5)

            logger.info("\n" + "=" * 60)
            if all_success:
                logger.info("✓ ALL THRUSTERS TESTED SUCCESSFULLY")
                logger.info("All 8 thrusters are responding correctly")
            else:
                logger.warning("⚠ SOME THRUSTERS FAILED")
                logger.warning("Review errors above and check connections")
            logger.info("=" * 60)

            return all_success

        except KeyboardInterrupt:
            logger.warning("\n✗ Test interrupted by user")
            self.stop_all()
            return False

    def run(self):
        """Main test entry point."""
        # Connect to hardware
        if not self.connect():
            logger.error("Cannot start test without serial connection")
            return False

        try:
            # Run sequential test
            success = self.run_sequential_test()
            return success

        finally:
            # Always disconnect
            self.disconnect()


def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(
        description='Sequential thruster verification test'
    )
    parser.add_argument(
        '--port',
        default='/dev/ttyUSB0',
        help='Serial port (default: /dev/ttyUSB0)'
    )
    parser.add_argument(
        '--baud',
        type=int,
        default=115200,
        help='Baud rate (default: 115200)'
    )
    parser.add_argument(
        '--duration',
        type=float,
        default=1.0,
        help='Fire duration per thruster in seconds (default: 1.0)'
    )

    args = parser.parse_args()

    # Create and run test
    tester = ThrusterSequentialTest(port=args.port, baud=args.baud)
    tester.test_duration = args.duration

    success = tester.run()

    # Return appropriate exit code
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
