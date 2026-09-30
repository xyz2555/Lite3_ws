#!/usr/bin/env python3

import math
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from ros_gz_interfaces.msg import Contacts
from geometry_msgs.msg import PoseStamped


URDF_PATH = Path(
    "/home/lexion/lite3_ws/src/lite3_description/"
    "Lite3/urdf/Lite3_gazebo.urdf"
)

# Untuk standing statik, 50 ms cukup longgar.
# Nanti untuk locomotion kita turunkan.
CONTACT_SYNC_TOL = 0.050


def xyz_from_element(element, default=(0.0, 0.0, 0.0)):
    if element is None:
        return np.array(default, dtype=float)

    values = element.attrib.get("xyz", "")
    if not values:
        return np.array(default, dtype=float)

    return np.array(
        [float(v) for v in values.split()],
        dtype=float
    )


def rpy_from_element(element):
    if element is None:
        return np.zeros(3)

    values = element.attrib.get("rpy", "")
    if not values:
        return np.zeros(3)

    return np.array(
        [float(v) for v in values.split()],
        dtype=float
    )


def rpy_matrix(rpy):
    roll, pitch, yaw = rpy

    cr = math.cos(roll)
    sr = math.sin(roll)

    cp = math.cos(pitch)
    sp = math.sin(pitch)

    cy = math.cos(yaw)
    sy = math.sin(yaw)

    Rx = np.array([
        [1, 0, 0],
        [0, cr, -sr],
        [0, sr, cr],
    ])

    Ry = np.array([
        [cp, 0, sp],
        [0, 1, 0],
        [-sp, 0, cp],
    ])

    Rz = np.array([
        [cy, -sy, 0],
        [sy, cy, 0],
        [0, 0, 1],
    ])

    return Rz @ Ry @ Rx


def homogeneous(R, p):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)

    if norm < 1e-12:
        return np.eye(3)

    axis = axis / norm

    x, y, z = axis
    c = math.cos(angle)
    s = math.sin(angle)
    v = 1.0 - c

    return np.array([
        [
            x*x*v + c,
            x*y*v - z*s,
            x*z*v + y*s,
        ],
        [
            y*x*v + z*s,
            y*y*v + c,
            y*z*v - x*s,
        ],
        [
            z*x*v - y*s,
            z*y*v + x*s,
            z*z*v + c,
        ],
    ])


class Lite3URDFModel:

    def __init__(self, urdf_path):

        self.urdf_path = Path(urdf_path)

        root = ET.parse(self.urdf_path).getroot()

        self.links = {}
        self.joints = {}

        child_links = set()

        for link in root.findall("link"):

            name = link.attrib["name"]

            mass = 0.0
            com_xyz = np.zeros(3)

            inertial = link.find("inertial")

            if inertial is not None:

                mass_element = inertial.find("mass")

                if mass_element is not None:
                    mass = float(
                        mass_element.attrib["value"]
                    )

                origin = inertial.find("origin")

                com_xyz = xyz_from_element(origin)

            self.links[name] = {
                "mass": mass,
                "com_xyz": com_xyz,
            }

        for joint in root.findall("joint"):

            name = joint.attrib["name"]
            joint_type = joint.attrib["type"]

            parent = joint.find("parent").attrib["link"]
            child = joint.find("child").attrib["link"]

            origin = joint.find("origin")

            axis_element = joint.find("axis")

            if axis_element is not None:

                axis_values = axis_element.attrib.get(
                    "xyz",
                    "1 0 0"
                )

                axis = np.array(
                    [float(v) for v in axis_values.split()],
                    dtype=float
                )

            else:
                axis = np.array(
                    [1.0, 0.0, 0.0]
                )

            self.joints[name] = {
                "type": joint_type,
                "parent": parent,
                "child": child,
                "origin_xyz": xyz_from_element(origin),
                "origin_rpy": rpy_from_element(origin),
                "axis": axis,
            }

            child_links.add(child)

        # root link = link yang tidak menjadi child joint
        all_links = set(self.links.keys())

        root_candidates = list(
            all_links - child_links
        )

        if not root_candidates:
            raise RuntimeError(
                "Could not determine URDF root link"
            )

        self.root_link = root_candidates[0]

        self.children = {}

        for joint_name, joint in self.joints.items():

            parent = joint["parent"]

            if parent not in self.children:
                self.children[parent] = []

            self.children[parent].append(
                joint_name
            )

        self.total_mass = sum(
            link["mass"]
            for link in self.links.values()
        )

    def forward_link_transforms(self, joint_positions):

        transforms = {
            self.root_link: np.eye(4)
        }

        stack = [self.root_link]

        while stack:

            parent_link = stack.pop()

            T_parent = transforms[parent_link]

            for joint_name in self.children.get(
                parent_link,
                []
            ):

                joint = self.joints[joint_name]

                R_origin = rpy_matrix(
                    joint["origin_rpy"]
                )

                T_origin = homogeneous(
                    R_origin,
                    joint["origin_xyz"]
                )

                T_motion = np.eye(4)

                joint_type = joint["type"]

                if joint_type in (
                    "revolute",
                    "continuous"
                ):

                    q = joint_positions.get(
                        joint_name,
                        0.0
                    )

                    T_motion[:3, :3] = axis_rotation(
                        joint["axis"],
                        q
                    )

                elif joint_type == "prismatic":

                    q = joint_positions.get(
                        joint_name,
                        0.0
                    )

                    T_motion[:3, 3] = (
                        joint["axis"] * q
                    )

                T_child = (
                    T_parent
                    @ T_origin
                    @ T_motion
                )

                child_link = joint["child"]

                transforms[child_link] = T_child

                stack.append(child_link)

        return transforms

    def center_of_mass_in_base(
        self,
        joint_positions
    ):

        transforms = self.forward_link_transforms(
            joint_positions
        )

        total_mass = 0.0
        weighted = np.zeros(3)

        for link_name, link in self.links.items():

            mass = link["mass"]

            if mass <= 0:
                continue

            if link_name not in transforms:
                continue

            T_link = transforms[link_name]

            com_local = np.append(
                link["com_xyz"],
                1.0
            )

            com_base = (
                T_link @ com_local
            )[:3]

            weighted += mass * com_base
            total_mass += mass

        if total_mass <= 0:
            raise RuntimeError(
                "No inertial masses found in URDF"
            )

        return weighted / total_mass


class ContactEquilibrium(Node):

    def __init__(self):

        super().__init__(
            "lite3_contact_equilibrium"
        )

        self.lock = threading.Lock()

        self.legs = [
            "FL",
            "FR",
            "HL",
            "HR",
        ]

        base = (
            "/world/flat/model/lite3/link"
        )

        self.contact_topics = {

            "FL":
            f"{base}/FL_SHANK/"
            "sensor/FL_contact/contact",

            "FR":
            f"{base}/FR_SHANK/"
            "sensor/FR_contact/contact",

            "HL":
            f"{base}/HL_SHANK/"
            "sensor/HL_contact/contact",

            "HR":
            f"{base}/HR_SHANK/"
            "sensor/HR_contact/contact",
        }

        self.latest_contact = {
            leg: {
                "sim_time": None,
                "samples": [],
            }
            for leg in self.legs
        }

        self.joint_positions = {}

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            20
        )

        self.subscribers = []

        for leg, topic in self.contact_topics.items():

            self.subscribers.append(
                self.create_subscription(
                    Contacts,
                    topic,
                    lambda msg,
                    leg_name=leg:
                    self.contact_callback(
                        msg,
                        leg_name
                    ),
                    20
                )
            )

        # Current measured floating-base pose.
        #
        # Obtained previously from:
        #
        # gz model -m lite3 --pose
        #
        # XYZ:
        # 0.029007 -0.001997 0.342831
        #
        # RPY:
        # 0.002525 -0.001346 -0.001385
        if self.base_pose is None:
            return

        base_msg = self.base_pose

        base_xyz = np.array([
            base_msg.pose.position.x,
            base_msg.pose.position.y,
            base_msg.pose.position.z
        ])

        R_WB = self.quat_to_rotation_matrix(
            base_msg.pose.orientation
        )

        self.urdf = Lite3URDFModel(
            URDF_PATH
        )

        self.timer = self.create_timer(
            0.5,
            self.print_equilibrium
        )

        self.get_logger().info(
            "Lite3 static contact equilibrium analyzer started"
        )

        self.get_logger().info(
            f"URDF mass = "
            f"{self.urdf.total_mass:.6f} kg"
        )

        self.base_pose = None

        self.base_pose_sub = self.create_subscription(
        PoseStamped,
        '/lite3/base_pose',
        self.base_pose_callback,
        20
        )

    def base_pose_callback(self, msg):
        self.base_pose = msg

    def joint_callback(self, msg):

        with self.lock:

            for name, position in zip(
                msg.name,
                msg.position
            ):

                self.joint_positions[name] = (
                    float(position)
                )
    def quat_to_rotation_matrix(self, q):
        x = q.x
        y = q.y
        z = q.z
        w = q.w

        return np.array([
            [
                1 - 2*(y*y + z*z),
                2*(x*y - z*w),
                2*(x*z + y*w)
            ],
            [
                2*(x*y + z*w),
                1 - 2*(x*x + z*z),
                2*(y*z - x*w)
            ],
            [
                2*(x*z - y*w),
                2*(y*z + x*w),
                1 - 2*(x*x + y*y)
            ]
        ])
    def contact_callback(self, msg, leg):

        sim_time = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

        samples = []

        for contact in msg.contacts:

            n = min(
                len(contact.positions),
                len(contact.wrenches)
            )

            for i in range(n):

                position = contact.positions[i]

                wrench = (
                    contact.wrenches[i]
                    .body_1_wrench
                )

                force = wrench.force

                samples.append({
                    "r": np.array([
                        position.x,
                        position.y,
                        position.z,
                    ]),

                    "F": np.array([
                        force.x,
                        force.y,
                        force.z,
                    ]),
                })

        with self.lock:

            self.latest_contact[leg] = {
                "sim_time": sim_time,
                "samples": samples,
            }

    def current_base_transform(self):

        R = rpy_matrix(
            self.base_rpy
        )

        return homogeneous(
            R,
            self.base_xyz
        )

    def get_com_world(self):

        with self.lock:

            q = dict(
                self.joint_positions
            )

        if len(q) == 0:
            return None

        com_base = (
            self.urdf.center_of_mass_in_base(q)
        )

        T_base = (
            self.current_base_transform()
        )

        com_world = (
            T_base @
            np.append(
                com_base,
                1.0
            )
        )[:3]

        return com_world

    @staticmethod
    def convex_hull(points):

        if len(points) <= 1:
            return points

        pts = sorted(
            [(p[0], p[1]) for p in points]
        )

        def cross(o, a, b):

            return (
                (a[0] - o[0]) *
                (b[1] - o[1])
                -
                (a[1] - o[1]) *
                (b[0] - o[0])
            )

        lower = []

        for p in pts:

            while (
                len(lower) >= 2
                and cross(
                    lower[-2],
                    lower[-1],
                    p
                ) <= 0
            ):
                lower.pop()

            lower.append(p)

        upper = []

        for p in reversed(pts):

            while (
                len(upper) >= 2
                and cross(
                    upper[-2],
                    upper[-1],
                    p
                ) <= 0
            ):
                upper.pop()

            upper.append(p)

        return (
            lower[:-1]
            + upper[:-1]
        )

    @staticmethod
    def point_inside_polygon(
        point,
        polygon
    ):

        if len(polygon) < 3:
            return False

        x, y = point

        inside = False

        j = len(polygon) - 1

        for i in range(len(polygon)):

            xi, yi = polygon[i]
            xj, yj = polygon[j]

            intersects = (
                ((yi > y) != (yj > y))
                and
                (
                    x <
                    (xj - xi)
                    * (y - yi)
                    / (
                        (yj - yi)
                        + 1e-15
                    )
                    + xi
                )
            )

            if intersects:
                inside = not inside

            j = i

        return inside

    def print_equilibrium(self):

        snapshot_time = None

        with self.lock:

            for leg in self.legs:

                t = self.latest_contact[
                    leg
                ]["sim_time"]

                if t is not None:

                    if (
                        snapshot_time is None
                        or t > snapshot_time
                    ):
                        snapshot_time = t

        if snapshot_time is None:
            print(
                "\nWaiting for contact data..."
            )
            return

        forces = {}
        positions = {}

        active_legs = []

        with self.lock:

            for leg in self.legs:

                data = self.latest_contact[
                    leg
                ]

                t = data["sim_time"]

                if t is None:
                    continue

                time_error = (
                    snapshot_time - t
                )

                if (
                    time_error >
                    CONTACT_SYNC_TOL
                ):
                    continue

                if not data["samples"]:
                    continue

                F = np.zeros(3)
                r_sum = np.zeros(3)
                fz_sum = 0.0

                for sample in data["samples"]:

                    r = sample["r"]
                    force = sample["F"]

                    F += force

                    r_sum += (
                        r * force[2]
                    )

                    fz_sum += force[2]

                if abs(fz_sum) > 1e-12:

                    r_effective = (
                        r_sum / fz_sum
                    )

                else:

                    r_effective = (
                        np.mean(
                            [
                                s["r"]
                                for s
                                in data["samples"]
                            ],
                            axis=0
                        )
                    )

                forces[leg] = F
                positions[leg] = (
                    r_effective
                )

                active_legs.append(leg)

        if not active_legs:

            print(
                "\nNo synchronized contacts yet."
            )
            return

        total_force = np.zeros(3)

        for F in forces.values():
            total_force += F

        total_Fz = total_force[2]

        # Force-weighted contact point.
        if abs(total_Fz) > 1e-12:

            cop = np.zeros(3)

            for leg in active_legs:

                cop += (
                    positions[leg]
                    * forces[leg][2]
                )

            cop /= total_Fz

        else:

            cop = np.zeros(3)

        # COM
        try:

            com_world = (
                self.get_com_world()
            )

        except Exception as exc:

            self.get_logger().error(
                f"COM calculation failed: {exc}"
            )

            com_world = None

        # Moment of contact forces about COM.
        moment_about_com = np.zeros(3)

        if com_world is not None:

            for leg in active_legs:

                r = (
                    positions[leg]
                    - com_world
                )

                moment_about_com += (
                    np.cross(
                        r,
                        forces[leg]
                    )
                )

        print()
        print("=" * 100)
        print(
            "LITE3 STATIC CONTACT EQUILIBRIUM"
        )
        print("=" * 100)

        print(
            f"Snapshot simulation time: "
            f"{snapshot_time:.6f} s"
        )

        print(
            f"Active contacts: "
            f"{', '.join(active_legs)}"
        )

        print("-" * 100)

        for leg in self.legs:

            if leg not in forces:

                print(
                    f"{leg}: NO CONTACT"
                )

                continue

            F = forces[leg]
            r = positions[leg]

            load_share = (
                F[2] / total_Fz * 100
                if abs(total_Fz) > 1e-12
                else 0.0
            )

            print(
                f"{leg}: "
                f"F=["
                f"{F[0]:+.6e}, "
                f"{F[1]:+.6e}, "
                f"{F[2]:+.6f}"
                f"] N | "
                f"r=["
                f"{r[0]:+.6f}, "
                f"{r[1]:+.6f}, "
                f"{r[2]:+.6f}"
                f"] m | "
                f"load={load_share:.3f}%"
            )

        print("-" * 100)

        print(
            f"TOTAL FORCE = "
            f"["
            f"{total_force[0]:+.9f}, "
            f"{total_force[1]:+.9f}, "
            f"{total_force[2]:+.9f}"
            f"] N"
        )

        print(
            f"Resultant F magnitude = "
            f"{np.linalg.norm(total_force):.9f} N"
        )

        if (
            abs(total_Fz) > 1e-12
        ):

            print(
                f"Fz-weighted CoP = "
                f"["
                f"{cop[0]:+.6f}, "
                f"{cop[1]:+.6f}, "
                f"{cop[2]:+.6f}"
                f"] m"
            )

        expected_weight = (
            self.urdf.total_mass
            * 9.81
        )

        print(
            f"Robot mass = "
            f"{self.urdf.total_mass:.9f} kg"
        )

        print(
            f"Expected weight = "
            f"{expected_weight:.9f} N"
        )

        force_error = (
            total_Fz
            - expected_weight
        )

        force_error_pct = (
            abs(force_error)
            / expected_weight
            * 100.0
        )

        print(
            f"Vertical force residual = "
            f"{force_error:+.9e} N"
        )

        print(
            f"Vertical force error = "
            f"{force_error_pct:.9f} %"
        )

        if com_world is not None:

            print(
                f"COM world = "
                f"["
                f"{com_world[0]:+.6f}, "
                f"{com_world[1]:+.6f}, "
                f"{com_world[2]:+.6f}"
                f"] m"
            )

            print(
                f"COM horizontal projection = "
                f"["
                f"{com_world[0]:+.6f}, "
                f"{com_world[1]:+.6f}"
                f"] m"
            )

            print(
                f"Contact moment about COM = "
                f"["
                f"{moment_about_com[0]:+.9e}, "
                f"{moment_about_com[1]:+.9e}, "
                f"{moment_about_com[2]:+.9e}"
                f"] N.m"
            )

            print(
                f"|M_contact_about_COM| = "
                f"{np.linalg.norm(moment_about_com):.9e}"
                f" N.m"
            )

            com_xy = (
                com_world[0],
                com_world[1]
            )

            contact_xy = [
                (
                    positions[leg][0],
                    positions[leg][1]
                )
                for leg in active_legs
            ]

            hull = self.convex_hull(
                contact_xy
            )

            inside = (
                self.point_inside_polygon(
                    com_xy,
                    hull
                )
            )

            print(
                f"COM projection inside "
                f"support polygon: "
                f"{inside}"
            )

            if len(hull) >= 3:

                print(
                    "Support polygon vertices:"
                )

                for p in hull:

                    print(
                        f"  ({p[0]:+.6f}, "
                        f"{p[1]:+.6f})"
                    )

        print("=" * 100)


def main(args=None):

    rclpy.init(args=args)

    node = ContactEquilibrium()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()