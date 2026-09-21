#!/usr/bin/env python3

import math
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts


URDF_PATH = Path(
    "/home/lexion/lite3_ws/src/lite3_description/"
    "Lite3/urdf/Lite3_gazebo.urdf"
)

CONTACT_TIMEOUT = 0.10  # seconds, wall-clock time


def load_robot_mass(urdf_path: Path) -> float:
    """Sum all link inertial masses from the URDF."""
    root = ET.parse(urdf_path).getroot()

    total_mass = 0.0

    for mass_element in root.findall(".//mass"):
        value = mass_element.attrib.get("value")
        if value is not None:
            total_mass += float(value)

    return total_mass


class Lite3ContactMonitor(Node):

    def __init__(self):
        super().__init__("lite3_contact_monitor")

        self.lock = threading.Lock()

        self.legs = ["FL", "FR", "HL", "HR"]

        base_topic = "/world/flat/model/lite3/link"

        self.topics = {
            "FL": f"{base_topic}/FL_SHANK/sensor/FL_contact/contact",
            "FR": f"{base_topic}/FR_SHANK/sensor/FR_contact/contact",
            "HL": f"{base_topic}/HL_SHANK/sensor/HL_contact/contact",
            "HR": f"{base_topic}/HR_SHANK/sensor/HR_contact/contact",
        }

        # Last received data for each leg
        self.latest = {
            leg: {
                "rx_time": None,
                "sim_time": None,
                "contacts": [],
            }
            for leg in self.legs
        }

        # Subscribers
        self.subscribers = []

        for leg, topic in self.topics.items():
            sub = self.create_subscription(
                Contacts,
                topic,
                lambda msg, leg_name=leg:
                    self.contact_callback(msg, leg_name),
                10,
            )
            self.subscribers.append(sub)

        # Robot mass from URDF
        try:
            self.robot_mass = load_robot_mass(URDF_PATH)
        except Exception as exc:
            self.robot_mass = 0.0
            self.get_logger().warning(
                f"Could not read robot mass: {exc}"
            )

        self.timer = self.create_timer(
            0.2,
            self.print_snapshot
        )

        self.get_logger().info(
            "Lite3 contact monitor started"
        )

        self.get_logger().info(
            f"Robot mass from URDF: "
            f"{self.robot_mass:.6f} kg"
        )

    def contact_callback(self, msg: Contacts, leg: str):

        samples = []

        # Contacts may contain multiple Contact objects.
        for contact in msg.contacts:

            collision1 = contact.collision1.name
            collision2 = contact.collision2.name

            # Only use contacts involving the ground.
            ground_name = "ground::ground_link::ground_collision"

            if (
                ground_name not in collision1
                and ground_name not in collision2
            ):
                continue

            n = min(
                len(contact.positions),
                len(contact.wrenches)
            )

            for i in range(n):

                position = contact.positions[i]

                # body_1 is Lite3 because collision1 is the foot
                # in the contact messages observed so far.
                wrench = contact.wrenches[i].body_1_wrench

                force = wrench.force

                samples.append(
                    {
                        "px": position.x,
                        "py": position.y,
                        "pz": position.z,

                        "fx": force.x,
                        "fy": force.y,
                        "fz": force.z,
                    }
                )

        sim_time = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

        with self.lock:
            self.latest[leg] = {
                "rx_time": time.monotonic(),
                "sim_time": sim_time,
                "contacts": samples,
            }

    def print_snapshot(self):

        now = time.monotonic()

        total_fx = 0.0
        total_fy = 0.0
        total_fz = 0.0

        # Force-weighted contact location
        weighted_x = 0.0
        weighted_y = 0.0

        print()
        print("=" * 88)
        print("LITE3 CONTACT SNAPSHOT")
        print("=" * 88)

        active_legs = []

        with self.lock:

            for leg in self.legs:

                data = self.latest[leg]

                if data["rx_time"] is None:
                    print(
                        f"{leg}: NO DATA"
                    )
                    continue

                age = now - data["rx_time"]

                # No fresh contact message
                if age > CONTACT_TIMEOUT:
                    print(
                        f"{leg}: NO CONTACT "
                        f"(last update {age:.3f} s ago)"
                    )
                    continue

                samples = data["contacts"]

                # Message received but contains no contact.
                if not samples:
                    print(
                        f"{leg}: NO CONTACT"
                    )
                    continue

                active_legs.append(leg)

                leg_fx = 0.0
                leg_fy = 0.0
                leg_fz = 0.0

                for s in samples:

                    leg_fx += s["fx"]
                    leg_fy += s["fy"]
                    leg_fz += s["fz"]

                    total_fx += s["fx"]
                    total_fy += s["fy"]
                    total_fz += s["fz"]

                    weighted_x += s["px"] * s["fz"]
                    weighted_y += s["py"] * s["fz"]

                print(
                    f"{leg}: CONTACT | "
                    f"F = ["
                    f"{leg_fx:+.6f}, "
                    f"{leg_fy:+.6f}, "
                    f"{leg_fz:+.6f}"
                    f"] N | "
                    f"contacts = {len(samples)} | "
                    f"age = {age:.4f} s"
                )

        print("-" * 88)

        print(
            f"ACTIVE LEGS: "
            f"{', '.join(active_legs) if active_legs else 'NONE'}"
        )

        print(
            f"TOTAL FORCE = ["
            f"{total_fx:+.6f}, "
            f"{total_fy:+.6f}, "
            f"{total_fz:+.6f}"
            f"] N"
        )

        if total_fz > 1e-9:

            cop_x = weighted_x / total_fz
            cop_y = weighted_y / total_fz

            print(
                f"Fz-weighted contact point = ["
                f"{cop_x:+.6f}, "
                f"{cop_y:+.6f}, "
                f"0.000000"
                f"] m"
            )

        if self.robot_mass > 0.0:

            expected_weight = self.robot_mass * 9.81

            force_error = total_fz - expected_weight

            force_error_pct = (
                abs(force_error)
                / expected_weight
                * 100.0
            )

            print(
                f"Robot mass = "
                f"{self.robot_mass:.6f} kg"
            )

            print(
                f"Expected weight = "
                f"{expected_weight:.6f} N"
            )

            print(
                f"Fz balance error = "
                f"{force_error:+.6f} N "
                f"({force_error_pct:.4f} %)"
            )

        print("=" * 88)


def main(args=None):

    rclpy.init(args=args)

    node = Lite3ContactMonitor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()