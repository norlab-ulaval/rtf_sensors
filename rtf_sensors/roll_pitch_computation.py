import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped, PointStamped
import math
from sympy import symbols, Eq, solve


class PlaneOrientationNode(Node):

    def __init__(self):
        super().__init__('plane_orientation_node')

        # Declare and get parameters
        self.declare_parameter('d_ab', 0.5)
        self.declare_parameter('d_ac', 0.6)
        self.declare_parameter('d_bc', 0.5)

        self.d_ab = self.get_parameter('d_ab').get_parameter_value().double_value
        self.d_ac = self.get_parameter('d_ac').get_parameter_value().double_value
        self.d_bc = self.get_parameter('d_bc').get_parameter_value().double_value

        self.get_logger().info(f'Using distances: AB={self.d_ab}, AC={self.d_ac}, BC={self.d_bc}')

        # Altitudes
        self.z1 = None
        self.z2 = None
        self.z3 = None

        self.header = None

        # Subscribers
        self.create_subscription(PointStamped, '/altitude_a', self.alt_a_callback, 10)
        self.create_subscription(PointStamped, '/altitude_b', self.alt_b_callback, 10)
        self.create_subscription(PointStamped, '/altitude_c', self.alt_c_callback, 10)

        # Publisher
        self.orientation_pub = self.create_publisher(Vector3Stamped, '/plane_orientation', 10)

    def alt_a_callback(self, msg):
        self.z1 = msg.point.z
        self.header = msg.header
        self.try_compute()

    def alt_b_callback(self, msg):
        self.z2 = msg.point.z
        self.try_compute()

    def alt_c_callback(self, msg):
        self.z3 = msg.point.z
        self.try_compute()

    def try_compute(self):
        if self.z1 is not None and self.z2 is not None and self.z3 is not None:
            try:
                roll, pitch = self.compute_orientation()
                msg = Vector3Stamped()
                msg.header = self.header
                msg.x = roll
                msg.y = pitch
                msg.z = 0.0
                self.orientation_pub.publish(msg)
                self.get_logger().info(f'Published Roll: {roll:.2f}, Pitch: {pitch:.2f}')
            except Exception as e:
                self.get_logger().error(f'Failed to compute orientation: {e}')

    def compute_orientation(self):
        x, y = symbols('x y')
        eq1 = Eq(x**2 + y**2 + (self.z3 - self.z1)**2, self.d_ac**2)
        eq2 = Eq((x - self.d_ab)**2 + y**2 + (self.z3 - self.z2)**2, self.d_bc**2)
        solutions = solve((eq1, eq2), (x, y), dict=True)

        for sol in solutions:
            if all(val.is_real for val in sol.values()):
                x_val = float(sol[x])
                y_val = float(sol[y])
                break
        else:
            raise ValueError('No valid solution for triangle geometry')

        # Normal vector
        n_x = -y_val * (self.z2 - self.z1)
        n_y = x_val * (self.z2 - self.z1) - self.d_ab * (self.z3 - self.z1)
        n_z = self.d_ab * y_val

        norm = math.sqrt(n_x**2 + n_y**2 + n_z**2)
        nx, ny, nz = n_x / norm, n_y / norm, n_z / norm

        roll = math.degrees(math.atan2(ny, nz))
        pitch = math.degrees(math.atan2(-nx, math.sqrt(ny**2 + nz**2)))

        return roll, pitch


def main(args=None):
    rclpy.init(args=args)
    node = PlaneOrientationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
