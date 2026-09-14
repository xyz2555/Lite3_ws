#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Lite3FKValidator(Node):
    def __init__(self):
        super().__init__('lite3_fk_validator')

        self.joint_names = [
            'FL_HipX_joint', 'FL_HipY_joint', 'FL_Knee_joint',
            'FR_HipX_joint', 'FR_HipY_joint', 'FR_Knee_joint',
            'HL_HipX_joint', 'HL_HipY_joint', 'HL_Knee_joint',
            'HR_HipX_joint', 'HR_HipY_joint', 'HR_Knee_joint'
        ]

        # Lite3 geometry from the validated URDF chain
        self.legs = {
            'FL': {
                'hip_x':  0.1745,
                'hip_y':  0.0620,
                'd':       0.09735,
            },
            'FR': {
                'hip_x':  0.1745,
                'hip_y': -0.0620,
                'd':      -0.09735,
            },
            'HL': {
                'hip_x': -0.1745,
                'hip_y':  0.0620,
                'd':       0.09735,
            },
            'HR': {
                'hip_x': -0.1745,
                'hip_y': -0.0620,
                'd':      -0.09735,
            },
        }

        self.L1 = 0.20000
        self.L2 = 0.21012

        self.q = {name: 0.0 for name in self.joint_names}

        self.received = False

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.timer = self.create_timer(1.0, self.print_fk)

        self.get_logger().info('Lite3 FK validator started.')

    def joint_state_callback(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):
            if name in self.q:
                self.q[name] = position

        self.received = True

    def foot_fk(self, leg):
        geom = self.legs[leg]

        q1 = self.q[f'{leg}_HipX_joint']
        q2 = self.q[f'{leg}_HipY_joint']
        q3 = self.q[f'{leg}_Knee_joint']

        a = (
            self.L1 * math.sin(q2)
            + self.L2 * math.sin(q2 + q3)
        )

        b = (
            -self.L1 * math.cos(q2)
            -self.L2 * math.cos(q2 + q3)
        )

        x = geom['hip_x'] + a

        y = (
            geom['hip_y']
            + geom['d'] * math.cos(q1)
            + b * math.sin(q1)
        )

        z = (
            -geom['d'] * math.sin(q1)
            + b * math.cos(q1)
        )

        return x, y, z

    def print_fk(self):
        if not self.received:
            self.get_logger().info('Waiting for /joint_states...')
            return

        self.get_logger().info('----- Lite3 Foot FK -----')

        for leg in ['FL', 'FR', 'HL', 'HR']:
            x, y, z = self.foot_fk(leg)

            self.get_logger().info(
                f'{leg}: '
                f'x={x:+.6f} m, '
                f'y={y:+.6f} m, '
                f'z={z:+.6f} m'
            )


def main(args=None):
    rclpy.init(args=args)

    node = Lite3FKValidator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()