#! /usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import Int16

from controllers.controller_lqr_2 import LQRController


def distance_to_point(a, b, c, x0, y0):
    def distance(x):
        y = a * x**2 + b * x + c
        return np.sqrt((x - x0) ** 2 + (y - y0) ** 2)

    from scipy.optimize import minimize

    result = minimize(distance, 0)
    return result.x[0], result.fun


def angle_with_x_axis(a, b, x0):
    derivative = 2 * a * x0 + b
    return np.degrees(np.arctan(derivative))


def angle_with_y_axis(a, b, x0):
    # Calculate derivative at x0

    derivative = 2 * a * x0 + b
    f"Berechnetes ableitung {derivative}"
    # Calculate angle with y-axis in degrees (90° - angle with x-axis)
    angle = np.pi / 2 - np.arctan(derivative)
    return angle


class LateralControllerNodeLQR_4(Node):
    def __init__(self):
        super().__init__("lateral_controller_node_lqr_4")

        self.get_logger().info("LQR Node initialized – Hallo!")

        self.lqr_controller = LQRController()
        # self.speed = 0.2  # fest angenommen wie im MATLAB-Skript

        self.publisher = self.create_publisher(
            Int16, "/control/steering_angle/target", 10
        )

        self.left_polinom_subscription = self.create_subscription(
            Vector3, "/path_planning/target/left", self.path_callback, 10
        )

    def path_callback(self, msg):
        a = msg.x
        b = msg.y
        c = msg.z

        x_closest, offset = distance_to_point(a, b, c, 0, 0)
        offset = offset / 1000
        angle_delta = angle_with_x_axis(a, b, 0)

        # Krümmung näherungsweise aus 2. Ableitung
        if b > 0.001:
            curvature = 1 / 1.5
            angle_delta = angle_delta * -1
        elif b < -0.01:
            curvature = -1 / 1.5
        else:
            curvature = 0
        control = self.lqr_controller.get_control_signal(
            [angle_delta, offset], curvature
        )

        control = int(control)
        self.get_logger().info(
            f"Berechnetes u (Lenkwinkel): {control}°, delta={angle_delta:.2f}, offset={offset:.3f}, k={curvature:.3f}, a{a}, b{b}, c{c}"
        )
        if curvature > 0:
            control = control * -1
        msg_out = Int16()
        msg_out.data = control
        self.publisher.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = LateralControllerNodeLQR_4()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# [INFO] [1743172266.662757035] [lateral_controller_node_lqr_1]: Berechnetes u (Lenkwinkel): -9°, delta=-14.59, offset=0.145, k=-0.667, a-0.00025768091770846655, b-0.2603788278871839, c150.71475396352946
# [INFO] [1743172266.697408639] [lateral_controller_node_lqr_1]: Berechnetes u (Lenkwinkel): -9°, delta=-14.59, offset=0.145, k=-0.667, a-0.00025768091770846655, b-0.2603788278871839, c150.71475396352946
# [INFO] [1743172266.779352204] [lateral_controller_node_lqr_1]: Berechnetes u (Lenkwinkel): -7°, delta=-13.85, offset=0.149, k=-0.667, a-0.0002672652928562974, b-0.24656573899965, c153.6917041616909
# [INFO] [1743172266.818546424] [lateral_controller_node_lqr_1]: Berechnetes u (Lenkwinkel): -7°, delta=-13.85, offset=0.149, k=-0.667, a-0.0002672652928562974, b-0.24656573899965, c153.6917041616909
# [INFO] [1743172266.923129052] [lateral_controller_node_lqr_1]: Berechnetes u (Lenkwinkel): -8°, delta=-14.51, offset=0.148, k=-0.667, a-0.00027101411766052334, b-0.2587400129341444, c153.597610820911
# [INFO] [1743172266.957668663] [lateral_controller_node_lqr_1]: Berechnetes u (Lenkwinkel): -8°, delta=-14.51, offset=0.148, k=-0.667, a-0.00027101411766052334, b-0.2587400129341444, c153.597610820911
# [INFO] [1743172267.071257670] [lateral_controller_node_lqr_1]: Berechnetes u (Lenkwinkel): -8°, delta=-14.50, offset=0.149, k=-0.667, a-0.0002832666672286507, b-0.2585628540184682, c154.46994640028237
# [INFO] [1743172267.103270842] [lateral_controller_node_lqr_1]: Berechnetes u (Lenkwinkel): -8°, delta=-14.50, offset=0.149, k=-0.667, a-0.0002832666672286507, b-0.2585628540184682, c154.46994640028237
# [INFO] [1743172267.159977462] [lateral_controller_node_lqr_1]: Berechnetes u (Lenkwinkel): -10°, delta=-15.20, offset=0.140, k=-0.667, a-0.0002724937212740183, b-0.2717074681492091, c145.82911352452476
# [INFO] [1743172267.196203533] [lateral_controller_node_lqr_1]: Berechnetes u (Lenkwinkel): -10°, delta=-15.20, offset=0.140, k=-0.667, a-0.0002724937212740183, b-0.2717074681492091, c145.82911352452476


# [INFO] [1743172168.815161823] [lateral_controller_node_lqr_1]: Berechnetes u (Lenkwinkel): 5°, delta=2.34, offset=0.014, k=0.000, a0.00016252962620071776, b0.04089901736128854, c-14.153777721399038
# [INFO] [1743172168.915114444] [lateral_controller_node_lqr_1]: Berechnetes u (Lenkwinkel): 7°, delta=2.51, offset=0.027, k=0.000, a-3.693817648848469e-05, b0.043817439519169596, c26.886755856324953
# [INFO] [1743172168.970271394] [lateral_controller_node_lqr_1]: Berechnetes u (Lenkwinkel): 7°, delta=2.51, offset=0.027, k=0.000, a-3.693817648848469e-05, b0.043817439519169596, c26.886755856324953
# [INFO] [1743172169.017210443] [lateral_controller_node_lqr_1]: Berechnetes u (Lenkwinkel): 45°, delta=21.32, offset=0.171, k=0.667, a-9.017285371708749e-05, b0.390285066463339, c183.4186229539891
# [INFO] [1743172169.163015319] [lateral_controller_node_lqr_1]: Berechnetes u (Lenkwinkel): 45°, delta=41.36, offset=0.188, k=0.667, a0.0003785903405490708, b0.8802962875278423, c245.19431402093954
# [INFO] [1743172169.209168072] [lateral_controller_node_lqr_1]: Berechnetes u (Lenkwinkel): 45°, delta=41.36, offset=0.188, k=0.667, a0.0003785903405490708, b0.8802962875278423, c245.19431402093954
# [INFO] [1743172169.286047814] [lateral_controller_node_lqr_1]: Berechnetes u (Lenkwinkel): 45°, delta=38.64, offset=0.322, k=0.667, a-0.0007944312770382146, b0.7993441558238418, c452.3610834223962
# [INFO] [1743172169.333246735] [lateral_controller_node_lqr_1]: Berechnetes u (Lenkwinkel): 45°, delta=38.64, offset=0.322, k=0.667, a-0.0007944312770382146, b0.7993441558238418, c452.3610834223962
# [INFO] [1743172169.445510447] [lateral_controller_node_lqr_1]: Berechnetes u (Lenkwinkel): 45°, delta=25.60, offset=0.529, k=0.667, a0.0002923132560315849, b0.479216297762263, c574.433856177396
# [INFO] [1743172169.493548819] [lateral_controller_node_lqr_1]: Berechnetes u (Lenkwinkel): 45°, delta=25.60, offset=0.529, k=0.667, a0.0002923132560315849, b0.479216297762263, c574.433856177396
# [INFO] [1743172169.528247751] [lateral_controller_node_lqr_1]: Berechnetes u (Lenkwinkel): 45°, delta=64.90, offset=0.596, k=0.667, a0.0018067770362632733, b2.1344757310912867, c1009.3233052087147
