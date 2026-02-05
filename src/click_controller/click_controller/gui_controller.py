import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tkinter as tk


class ClickController(Node):

    def __init__(self):
        super().__init__('click_controller')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd = Twist()

        self.timer = self.create_timer(0.1, self.publish_cmd)

        self.window = tk.Tk()
        self.window.title("Click Controller")

        self.canvas_size = 400
        self.canvas = tk.Canvas(
            self.window,
            width=self.canvas_size,
            height=self.canvas_size,
            bg="white"
        )
        self.canvas.pack()

        self.canvas.bind("<Button-1>", self.on_drag)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)

        c = self.canvas_size // 2
        self.canvas.create_line(c, 0, c, self.canvas_size, fill="gray")
        self.canvas.create_line(0, c, self.canvas_size, c, fill="gray")

        self.max_speed = 1.5
        self.max_turn = 3.0

        self.get_logger().info("GUI controller started")

    def on_drag(self, event):
        x = event.x - self.canvas_size / 2
        y = (self.canvas_size / 2) - event.y

        x_norm = x / (self.canvas_size / 2)
        y_norm = y / (self.canvas_size / 2)

        x_norm = max(min(x_norm, 1.0), -1.0)
        y_norm = max(min(y_norm, 1.0), -1.0)

        self.cmd.linear.x = -self.max_speed * y_norm
        self.cmd.angular.z = -self.max_turn * x_norm

    def on_release(self, event):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0

    def publish_cmd(self):
        self.publisher_.publish(self.cmd)

    def spin_gui(self):
        while rclpy.ok():
            self.window.update()
            rclpy.spin_once(self, timeout_sec=0.01)


def main():
    rclpy.init()
    node = ClickController()
    node.spin_gui()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
