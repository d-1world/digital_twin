import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage

import tkinter as tk
from PIL import Image, ImageTk
import cv2
import numpy as np
import threading

class GuiNode(Node):
    def __init__(self, gui_callback_lane_state, gui_callback_image):
        super().__init__('gui_node')

        # 퍼블리셔
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.max_vel_pub = self.create_publisher(Float64, '/control/max_vel', 10)

        # 서브스크라이버
        self.lane_state_sub = self.create_subscription(Int32, '/detect/lane_state', self.lane_state_callback, 10)
        self.image_sub = self.create_subscription(CompressedImage, '/detect/image_lane/compressed', self.image_callback, 10)

        self.gui_callback_lane_state = gui_callback_lane_state
        self.gui_callback_image = gui_callback_image

    def publish_cmd_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(f'Published cmd_vel: linear={linear}, angular={angular}')

    def publish_max_vel(self, value):
        msg = Float64()
        msg.data = value
        self.max_vel_pub.publish(msg)
        self.get_logger().info(f'Published max_vel: {value}')

    def lane_state_callback(self, msg):
        if self.gui_callback_lane_state:
            self.gui_callback_lane_state(msg.data)

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is None:
                self.get_logger().warn("cv_image is None after decoding")
                return

            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(cv_image)
            if self.gui_callback_image:
                self.gui_callback_image(pil_image)
        except Exception as e:
            self.get_logger().error(f"이미지 처리 실패: {e}")

class GUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("ROS 2 Control GUI")
        self.root.geometry("800x800")

        # 선속도 슬라이더
        self.linear_scale = tk.Scale(self.root, from_=-2.0, to=2.0, resolution=0.1,
                                     orient="horizontal", label="선속도 (linear.x)")
        self.linear_scale.set(0.5)
        self.linear_scale.pack()

        # 각속도 슬라이더
        self.angular_scale = tk.Scale(self.root, from_=-3.14, to=3.14, resolution=0.1,
                                      orient="horizontal", label="각속도 (angular.z)")
        self.angular_scale.set(0.0)
        self.angular_scale.pack()

        # 속도 명령 퍼블리시 버튼
        self.publish_vel_button = tk.Button(self.root, text="속도 명령 퍼블리시", command=self.on_publish_cmd_vel)
        self.publish_vel_button.pack()

        # 최고 속도 슬라이더
        self.max_vel_scale = tk.Scale(self.root, from_=0, to=5, resolution=0.1,
                                      orient="horizontal", label="최고 속도")
        self.max_vel_scale.set(1.0)
        self.max_vel_scale.pack()

        # 최고 속도 퍼블리시 버튼
        self.max_vel_button = tk.Button(self.root, text="최고 속도 설정 퍼블리시", command=self.on_publish_max_vel)
        self.max_vel_button.pack()

        # 라벨: 선 개수
        self.lane_state_label = tk.Label(self.root, text="Lane State: 없음")
        self.lane_state_label.pack(pady=10)

        # 이미지 표시 라벨
        self.image_label = tk.Label(self.root)
        self.image_label.pack()

        self.ros_node = None

    def set_ros_node(self, node):
        self.ros_node = node

    def on_publish_cmd_vel(self):
        if self.ros_node:
            linear = self.linear_scale.get()
            angular = self.angular_scale.get()
            self.ros_node.publish_cmd_vel(linear, angular)

    def on_publish_max_vel(self):
        if self.ros_node:
            value = self.max_vel_scale.get()
            self.ros_node.publish_max_vel(value)

    def update_lane_state(self, count):
        self.lane_state_label.after(0, lambda: self.lane_state_label.config(text=f"Lane State: {count}개"))

    def update_image(self, image):
        tk_img = ImageTk.PhotoImage(image)
        self.image_label.after(0, lambda: self.image_label.config(image=tk_img))
        self.image_label.image = tk_img  # prevent GC

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)

    gui = GUI()
    node = GuiNode(gui.update_lane_state, gui.update_image)
    gui.set_ros_node(node)

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    gui.run()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
