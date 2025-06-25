import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage

import tkinter as tk
from PIL import Image, ImageTk
import cv2
import numpy as np
import math
import threading

import os

class GuiNode(Node):
    def __init__(self, gui_callback_lane_state, gui_callback_image):
        super().__init__('gui_node')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.max_vel_pub = self.create_publisher(Float64, '/control/max_vel', 10)

        self.lane_state_sub = self.create_subscription(Int32, '/detect/lane_state', self.lane_state_callback, 10)
        self.image_sub = self.create_subscription(CompressedImage, '/detect/image_lane/compressed', self.image_callback, 10)

        self.gui_callback_lane_state = gui_callback_lane_state
        self.gui_callback_image = gui_callback_image

    def publish_cmd_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(f'Published cmd_vel: linear={linear:.2f}, angular={angular:.2f}')

    def publish_max_vel(self, value):
        msg = Float64()
        msg.data = value
        self.max_vel_pub.publish(msg)
        self.get_logger().info(f'Published max_vel: {value:.2f}')

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




class SteeringHandle:
    def __init__(self, parent, on_angle_change):
        self.canvas = tk.Canvas(parent, width=200, height=200, bg='white', highlightthickness=0)
        self.canvas.pack(pady=10)
        self.center = (100, 100)
        self.radius = 80
        self.angle = 0.0
        self.on_angle_change = on_angle_change

        # 핸들 이미지 절대 경로 (home 디렉토리 포함)
        resource_path = os.path.expanduser('~/rokey_ws/src/digital_twin/resource/handle.png')
        self.handle_img_orig = Image.open(resource_path).resize((160, 160))
        self.handle_img = ImageTk.PhotoImage(self.handle_img_orig)
        self.handle_image_id = self.canvas.create_image(self.center[0], self.center[1], image=self.handle_img)

        self.canvas.bind("<B1-Motion>", self.on_drag)

    def on_drag(self, event):
        dx = event.x - self.center[0]
        dy = event.y - self.center[1]

        angle_rad = math.atan2(-dy, dx) - math.pi / 2  # 위쪽 기준

        # -π ~ π 보정
        if angle_rad > math.pi:
            angle_rad -= 2 * math.pi
        elif angle_rad < -math.pi:
            angle_rad += 2 * math.pi

        self.angle = angle_rad

        # 이미지 회전 (시계방향이 양수, PIL은 반시계 방향이 양수)
        # rotated_img = self.handle_img_orig.rotate(-math.degrees(angle_rad), resample=Image.BICUBIC)
        rotated_img = self.handle_img_orig.rotate(math.degrees(angle_rad), resample=Image.BICUBIC)
        self.handle_img = ImageTk.PhotoImage(rotated_img)
        self.canvas.itemconfig(self.handle_image_id, image=self.handle_img)

        if self.on_angle_change:
            self.on_angle_change(angle_rad)


class GUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("ROS 2 Control GUI")
        self.root.geometry("800x950")

        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0

        self.speed_label = tk.Label(self.root, text="선속도: 0.0 m/s", font=("Arial", 14))
        self.speed_label.pack(pady=10)

        accel_brake_frame = tk.Frame(self.root)
        accel_brake_frame.pack(pady=5)

        self.accel_button = tk.Button(accel_brake_frame, text="엑셀 ↑", width=10, command=self.on_accel)
        self.accel_button.grid(row=0, column=0, padx=10)

        self.brake_button = tk.Button(accel_brake_frame, text="브레이크 ↓", width=10, command=self.on_brake)
        self.brake_button.grid(row=0, column=1, padx=10)

        self.angular_label = tk.Label(self.root, text="각속도: 0.0 rad/s", font=("Arial", 14))
        self.angular_label.pack(pady=10)

        self.steering = SteeringHandle(self.root, self.on_handle_angle_change)

        self.max_vel_scale = tk.Scale(self.root, from_=0, to=5, resolution=0.1,
                                      orient="horizontal", label="최고 속도")
        self.max_vel_scale.set(1.0)
        self.max_vel_scale.pack()

        self.max_vel_button = tk.Button(self.root, text="최고 속도 설정 퍼블리시", command=self.on_publish_max_vel)
        self.max_vel_button.pack(pady=10)

        self.stop_button = tk.Button(self.root, text="정지 ⏹", fg="red", width=20, command=self.on_stop)
        self.stop_button.pack(pady=10)

        self.lane_state_label = tk.Label(self.root, text="Lane State: 없음")
        self.lane_state_label.pack(pady=10)

        self.image_label = tk.Label(self.root)
        self.image_label.pack()

        self.ros_node = None

    def set_ros_node(self, node):
        self.ros_node = node

    def update_speed_label(self):
        self.speed_label.config(text=f"선속도: {self.current_linear_speed:.1f} m/s")

    def update_angular_label(self):
        self.angular_label.config(text=f"각속도: {self.current_angular_speed:.1f} rad/s")

    def on_accel(self):
        self.current_linear_speed = min(self.current_linear_speed + 0.1, 2.0)
        self.update_speed_label()
        self.publish_cmd()

    def on_brake(self):
        self.current_linear_speed = max(self.current_linear_speed - 0.1, -2.0)
        self.update_speed_label()
        self.publish_cmd()

    def on_handle_angle_change(self, angle_rad):
        self.current_angular_speed = max(min(angle_rad, 3.14), -3.14)
        self.update_angular_label()
        self.publish_cmd()

    def on_stop(self):
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.update_speed_label()
        self.update_angular_label()
        self.publish_cmd()

    def publish_cmd(self):
        if self.ros_node:
            self.ros_node.publish_cmd_vel(self.current_linear_speed, self.current_angular_speed)

    def on_publish_max_vel(self):
        if self.ros_node:
            value = self.max_vel_scale.get()
            self.ros_node.publish_max_vel(value)

    def update_lane_state(self, count):
        self.lane_state_label.after(0, lambda: self.lane_state_label.config(text=f"Lane State: {count}개"))

    def update_image(self, image):
        tk_img = ImageTk.PhotoImage(image)
        self.image_label.after(0, lambda: self.image_label.config(image=tk_img))
        self.image_label.image = tk_img

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
