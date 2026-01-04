"""
Real-time LD06 LIDAR point cloud visualisation.

Reads serial data from ESP32 and displays a polar plot of the LIDAR scan.
"""

import sys
import math
import serial
import serial.tools.list_ports
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque


class LidarVisualiser:
    """Real-time LIDAR point cloud visualiser."""

    def __init__(self, port: str, baud: int = 921600):
        self.port = port
        self.baud = baud
        self.ser = None

        # Store one full rotation of points (360 degrees)
        # Using dict with angle as key for fast updates
        self.points = {}
        self.max_distance = 8000  # mm

        # For tracking scan rate
        self.point_count = 0

    def connect(self) -> bool:
        """Connect to the serial port."""
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
            print(f"Connected to {self.port} at {self.baud} baud")
            return True
        except serial.SerialException as e:
            print(f"Failed to connect: {e}")
            return False

    def read_points(self) -> None:
        """Read available points from serial."""
        if not self.ser:
            return

        try:
            while self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line or line.startswith("LD06"):
                    continue

                parts = line.split(',')
                if len(parts) != 3:
                    continue

                try:
                    angle = float(parts[0])
                    distance = int(parts[1])
                    confidence = int(parts[2])

                    # Quantise angle to 0.5 degree bins for cleaner display
                    angle_bin = round(angle * 2) / 2

                    if 0 <= distance <= self.max_distance:
                        self.points[angle_bin] = (distance, confidence)
                        self.point_count += 1

                except ValueError:
                    continue

        except serial.SerialException:
            pass

    def get_polar_data(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Convert points to polar plot format."""
        if not self.points:
            return np.array([]), np.array([]), np.array([])

        angles = []
        distances = []
        confidences = []

        for angle, (dist, conf) in self.points.items():
            angles.append(math.radians(angle))
            distances.append(dist)
            confidences.append(conf)

        return np.array(angles), np.array(distances), np.array(confidences)


def list_serial_ports() -> list[str]:
    """List available serial ports."""
    ports = serial.tools.list_ports.comports()
    return [p.device for p in ports]


def main():
    # Find serial port
    ports = list_serial_ports()

    if not ports:
        print("No serial ports found.")
        sys.exit(1)

    # Use provided port or show selection
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        print("Available ports:")
        for i, p in enumerate(ports):
            print(f"  {i}: {p}")

        if len(ports) == 1:
            port = ports[0]
            print(f"Using: {port}")
        else:
            try:
                idx = int(input("Select port number: "))
                port = ports[idx]
            except (ValueError, IndexError):
                print("Invalid selection.")
                sys.exit(1)

    # Create visualiser
    vis = LidarVisualiser(port)

    if not vis.connect():
        sys.exit(1)

    # Set up the plot
    plt.style.use('dark_background')
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='polar')

    # Configure polar plot
    ax.set_theta_zero_location('N')  # 0 degrees at top
    ax.set_theta_direction(-1)  # Clockwise
    ax.set_rlim(0, 4000)  # 4m radius default
    ax.set_title('LD06 LIDAR Point Cloud', pad=20)

    # Add distance rings labels
    ax.set_rticks([1000, 2000, 3000, 4000])
    ax.set_yticklabels(['1m', '2m', '3m', '4m'])

    # Initial scatter plot
    scatter = ax.scatter([], [], c=[], cmap='plasma', s=2, vmin=100, vmax=255)

    # Add colorbar for confidence
    cbar = plt.colorbar(scatter, ax=ax, pad=0.1, shrink=0.8)
    cbar.set_label('Confidence')

    # Point count text
    count_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                         fontsize=10, verticalalignment='top')

    def update(frame):
        """Animation update function."""
        vis.read_points()

        angles, distances, confidences = vis.get_polar_data()

        if len(angles) > 0:
            scatter.set_offsets(np.c_[angles, distances])
            scatter.set_array(confidences)

            # Auto-scale if needed
            max_dist = np.max(distances)
            if max_dist > ax.get_rmax():
                ax.set_rlim(0, min(max_dist * 1.1, 8000))

        count_text.set_text(f'Points: {len(vis.points)}')

        return scatter, count_text

    # Create animation
    ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)

    print("Visualisation started. Close window to exit.")
    plt.tight_layout()
    plt.show()

    # Cleanup
    if vis.ser:
        vis.ser.close()


if __name__ == "__main__":
    main()
