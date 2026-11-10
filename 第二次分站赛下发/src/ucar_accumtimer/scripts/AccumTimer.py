#! /usr/bin/env python3.8
# -*- coding: utf-8 -*-

import tkinter as tk
from tkinter import ttk
import time
import rospy
from copy import deepcopy
from std_msgs.msg import Bool
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point32, PolygonStamped, Twist
import os

# ---------------- 配色 (Ubuntu Yaru 浅色) ----------------
BG      = "#f6f5f4"   # 窗口底色 (Yaru 窗口灰)
CARD    = "#ffffff"   # 卡片/面板底色
BORDER  = "#d9d9d9"   # 卡片描边
FG      = "#3d3d3d"   # 主文字 (Yaru 深灰)
MUTED   = "#767676"   # 次要文字
ACCENT  = "#e95420"   # Ubuntu 橙 (主按钮)
RED     = "#c01c28"   # 错误/停止
BLUE    = "#3584e4"   # 列表选中
BTN_BG  = "#f2f2f2"   # 普通按钮底色 (仿 Qt/rviz)
BTN_BD  = "#c7c7c7"   # 按钮描边
MONO    = "Ubuntu Mono"
SANS    = "Ubuntu"


class BoxOfArea:
    def __init__(self):
        # Area set
        self.global_frame_id = rospy.get_param(
            "~global_frame_id", "map")
        self.pub_Area_rectangle = []
        self.Area_rectangle = []

        # Area set
        self.Area_num = rospy.get_param("~Area_num", 1)
        point = Point32()
        point1 = []
        point2 = []
        point3 = []
        point4 = []
        self.x_max = []
        self.y_max = []
        self.x_min = []
        self.y_min = []

        for i in range(self.Area_num):
            self.pub_Area_rectangle.append(rospy.Publisher(
                "Area_rectangle"+str(i), PolygonStamped, queue_size=1))

            point.x = rospy.get_param('~Area_point1', [0.0, 0.0])[i][0]
            point.y = rospy.get_param('~Area_point1', [0.0, 0.0])[i][1]
            point1.append(deepcopy(point))
            point.x = rospy.get_param('~Area_point2', [0.0, 0.0])[i][0]
            point.y = rospy.get_param('~Area_point2', [0.0, 0.0])[i][1]
            point2.append(deepcopy(point))
            point.x = rospy.get_param('~Area_point1', [0.0, 0.0])[i][0]
            point.y = rospy.get_param('~Area_point2', [0.0, 0.0])[i][1]
            point3.append(deepcopy(point))
            point.x = rospy.get_param('~Area_point2', [0.0, 0.0])[i][0]
            point.y = rospy.get_param('~Area_point1', [0.0, 0.0])[i][1]
            point4.append(deepcopy(point))

            self.x_max.append(deepcopy(max(point1[i].x, point2[i].x)))
            self.x_min.append(deepcopy(min(point1[i].x, point2[i].x)))
            self.y_max.append(deepcopy(max(point1[i].y, point2[i].y)))
            self.y_min.append(deepcopy(min(point1[i].y, point2[i].y)))

            point_p = PolygonStamped()
            point_p.header.frame_id = self.global_frame_id
            point_p.polygon.points.append(point1[i])
            point_p.polygon.points.append(point3[i])
            point_p.polygon.points.append(point2[i])
            point_p.polygon.points.append(point4[i])

            self.Area_rectangle.append(point_p)

    def publish_Area_rectangle(self, area=-1):
        if area < 0:
            for i in range(self.Area_num):
                self.pub_Area_rectangle[i].publish(self.Area_rectangle[i])
        else:
            self.pub_Area_rectangle[area].publish(self.Area_rectangle[area])

    def is_point_in_area(self, point=Point32()):
        for i in range(self.Area_num):
            if point.x >= self.x_min[i] and \
                    point.x <= self.x_max[i] and \
                    point.y >= self.y_min[i] and \
                    point.y <= self.y_max[i]:
                return i
        return -1


class AccumTimerApp(object):
    def __init__(self, root):
        self.root = root
        self.root.title("ROS RACE TIMER")
        # 初始尺寸按屏幕自适应 (4K 屏上 800x600 只有一小块, 东西挤在一起)
        sw = self.root.winfo_screenwidth()
        sh = self.root.winfo_screenheight()
        self.win_w = max(1000, min(1500, int(sw * 0.5)))
        self.win_h = max(680, min(950, int(sh * 0.7)))
        self.root.geometry(f"{self.win_w}x{self.win_h}")

        self.group_number = None
        self.big_timer_started = False
        self.results = []
        self.small_timer_started = False
        self.small_timer_start_time = None

        # 创建ROS服务
        self.create_ros_services()

        # 速度判断标志
        self.zero_vel = False

        # 订阅速度话题
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        # 机器人位置: 直接订阅 gazebo 发布的真值位姿 (/gazebo/model_states,
        # 世界帧), 不再走 map→base_link 的 tf 查询。map 帧与 gazebo 世界帧
        # 重合 (map.yaml origin 补偿), 终点区域坐标可以直接比较
        self.gazebo_model_name = rospy.get_param("~gazebo_model_name", "car3")
        self.gazebo_index = None
        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         self.model_states_callback)
        self.position = Point32()
        self.end = -2
        self.end_box = BoxOfArea()
        self.global_frame_id = rospy.get_param("~global_frame_id", "map")

        # 创建用户界面
        self.create_gui()

        # 清空Result.md
        self.file_address = rospy.get_param("my_address")
        self.file = open(self.file_address + "Result.md", "w")
        self.file.write("")
        self.file.close()

        self.root.mainloop()

    # 速度回调函数
    def cmd_vel_callback(self, msg):
        if msg.linear.x <= 0.05 and msg.linear.y <= 0.01 and msg.linear.z <= 0.01:
            self.zero_vel = True
        else:
            self.zero_vel = False

    def _setup_styles(self):
        style = ttk.Style(self.root)
        style.theme_use("clam")
        # 字号固定 (窗口调大只多给布局留白, 不放大字)
        style.configure(".", background=BG, foreground=FG, font=(SANS, 10))
        style.configure("TFrame", background=BG)
        style.configure("Card.TFrame", background=CARD)
        style.configure("Title.TLabel", background=BG, foreground=FG,
                        font=(SANS, 15, "bold"))
        style.configure("Timer.TLabel", background=CARD, foreground="#262626",
                        font=(MONO, 76, "bold"))
        style.configure("Status.TLabel", background=CARD, foreground=FG,
                        font=(SANS, 16, "bold"))
        style.configure("SmallTimer.TLabel", background=BG, foreground=MUTED,
                        font=(MONO, 18, "bold"))
        style.configure("Field.TLabel", background=CARD, foreground=MUTED,
                        font=(SANS, 10))
        style.configure("Section.TLabel", background=CARD, foreground=FG,
                        font=(SANS, 12, "bold"))
        style.configure("Error.TLabel", background=CARD, foreground=RED,
                        font=(SANS, 10))
        # 主按钮: Ubuntu 橙
        style.configure("Accent.TButton", background=ACCENT, foreground="#ffffff",
                        font=(SANS, 11, "bold"), padding=(22, 9),
                        borderwidth=1, focusthickness=1,
                        bordercolor="#c74413", lightcolor="#c74413",
                        darkcolor="#c74413")
        style.map("Accent.TButton",
                  background=[("active", "#f0662e"), ("pressed", "#c74413")])
        # 普通按钮: 浅灰, 仿 Qt/rviz
        style.configure("Default.TButton", background=BTN_BG, foreground=FG,
                        font=(SANS, 11, "bold"), padding=(18, 9),
                        borderwidth=1, focusthickness=1,
                        bordercolor=BTN_BD, lightcolor=BTN_BD, darkcolor=BTN_BD)
        style.map("Default.TButton",
                  background=[("active", "#e8e8e8"), ("pressed", "#d5d5d5")])
        style.configure("Danger.TButton", background=BTN_BG, foreground=RED,
                        font=(SANS, 11, "bold"), padding=(22, 9),
                        borderwidth=1, focusthickness=1,
                        bordercolor=BTN_BD, lightcolor=BTN_BD, darkcolor=BTN_BD)
        style.map("Danger.TButton",
                  background=[("active", "#e8e8e8"), ("pressed", "#d5d5d5")])
        style.configure("TEntry", fieldbackground="#ffffff", foreground=FG,
                        insertcolor=FG, padding=7,
                        bordercolor=BTN_BD, lightcolor=BTN_BD, darkcolor=BTN_BD)
        style.map("TEntry",
                  bordercolor=[("focus", ACCENT)],
                  lightcolor=[("focus", ACCENT)],
                  darkcolor=[("focus", ACCENT)])

    # 给卡片包一圈 1px 灰边 (ttk.Frame 本身画不了边框)
    def _card(self, parent, row, column, sticky, padx, pady, rowspan=1,
              columnspan=1):
        border = tk.Frame(parent, bg=BORDER)
        border.grid(row=row, column=column, rowspan=rowspan,
                    columnspan=columnspan, sticky=sticky, padx=padx, pady=pady)
        inner = ttk.Frame(border, style="Card.TFrame")
        inner.pack(fill="both", expand=True, padx=1, pady=1)
        return inner

    def create_gui(self):
        self.root.configure(bg=BG)
        self.root.minsize(900, 620)
        self._setup_styles()

        self.root.grid_rowconfigure(1, weight=1)
        self.root.grid_rowconfigure(2, weight=1)
        self.root.grid_columnconfigure(0, weight=1, uniform="card")
        self.root.grid_columnconfigure(1, weight=1, uniform="card")

        # ---- 顶栏 ----
        header = ttk.Frame(self.root, style="TFrame")
        header.grid(row=0, column=0, columnspan=2, sticky="ew",
                    padx=24, pady=(16, 0))
        ttk.Label(header, text="ROS RACE TIMER",
                  style="Title.TLabel").pack(side="left")
        tk.Frame(header, bg=ACCENT, height=2).pack(side="bottom", fill="x")

        # ---- 计时面板 ----
        panel = self._card(self.root, 1, 0, "nsew", 24, 10, columnspan=2)
        panel.grid_rowconfigure(0, weight=1)
        panel.grid_rowconfigure(1, weight=1)
        panel.grid_columnconfigure(0, weight=1)
        self.big_timer_label = ttk.Label(
            panel, text="00:00:00", style="Timer.TLabel")
        self.big_timer_label.grid(row=0, sticky="s", pady=(28, 0))
        self.end_label = ttk.Label(panel, text="", style="Status.TLabel")
        self.end_label.grid(row=1, sticky="n", pady=(2, 20))

        # ---- 左卡片: 抽签控制 ----
        left = self._card(self.root, 2, 0, "nsew", (24, 10), (0, 10))
        left.configure(padding=(20, 16))
        left.grid_columnconfigure(0, weight=1)
        ttk.Label(left, text="DRAW", style="Section.TLabel").grid(
            row=0, column=0, sticky="w")
        self.group_entry_label = ttk.Label(
            left, text="Enter Group Number (no letter C):",
            style="Field.TLabel")
        self.group_entry_label.grid(row=1, column=0, sticky="w", pady=(14, 6))
        self.group_entry = ttk.Entry(left, width=14)
        self.group_entry.grid(row=2, column=0, sticky="ew", pady=(0, 12))
        btn_row = ttk.Frame(left, style="Card.TFrame")
        btn_row.grid(row=3, column=0, sticky="w")
        self.draw_button = ttk.Button(
            btn_row, text="Draw", style="Accent.TButton",
            command=self.draw_button_clicked)
        self.draw_button.pack(side="left")
        self.stop_button = ttk.Button(
            btn_row, text="Stop", style="Danger.TButton",
            command=self.stop_button_clicked)
        self.stop_button.pack(side="left", padx=(10, 0))
        self.group_error_label = ttk.Label(
            left, text="", style="Error.TLabel")
        self.group_error_label.grid(row=4, column=0, sticky="w", pady=(12, 0))

        # ---- 右卡片: 成绩 ----
        right = self._card(self.root, 2, 1, "nsew", (10, 24), (0, 10))
        right.configure(padding=(20, 16))
        right.grid_columnconfigure(0, weight=1)
        right.grid_rowconfigure(1, weight=1)
        self.result_label = ttk.Label(
            right, text="Results:", style="Section.TLabel")
        self.result_label.grid(row=0, column=0, sticky="w")
        self.result_listbox = tk.Listbox(
            right, width=20, height=10, bg="#ffffff", fg=FG,
            font=(MONO, 11), selectbackground=BLUE, selectforeground="#ffffff",
            relief="flat", highlightthickness=1, highlightbackground=BORDER,
            highlightcolor=ACCENT, borderwidth=0, activestyle="none")
        self.result_listbox.grid(row=1, column=0, sticky="nsew", pady=(12, 0))
        self.confirm_button = ttk.Button(
            right, text="Confirm the result", style="Default.TButton",
            command=self.confirm_button_clicked)
        self.confirm_button.grid(row=2, column=0, sticky="ew", pady=(12, 0))
        self.confirm_error_label = ttk.Label(
            right, text="", style="Error.TLabel")
        self.confirm_error_label.grid(row=3, column=0, sticky="w", pady=(10, 0))

        # ---- 底栏: 小计时器 ----
        footer = ttk.Frame(self.root, style="TFrame")
        footer.grid(row=3, column=0, columnspan=2, sticky="ew",
                    padx=24, pady=(0, 14))
        footer.grid_columnconfigure(0, weight=1)
        self.small_timer_label = ttk.Label(
            footer, text="00:00:00", style="SmallTimer.TLabel")
        self.small_timer_label.grid(row=0, column=1, sticky="e")

    def create_ros_services(self):
        # 初始化ROS节点并创建服务代理
        rospy.init_node('accum_timer')

    # 位姿回调: 从 /gazebo/model_states 里找 car3 的世界帧真值位姿
    def model_states_callback(self, msg):
        # 名字列表偶尔会变 (模型后到), 下标失效时重新查找
        if (self.gazebo_index is None or
                msg.name[self.gazebo_index] != self.gazebo_model_name):
            try:
                self.gazebo_index = msg.name.index(self.gazebo_model_name)
            except ValueError:
                return
        pose = msg.pose[self.gazebo_index]
        self.position.x = pose.position.x
        self.position.y = pose.position.y

    # Just as its name.
    def _get_robot_position(self):
        # 位置已由 model_states_callback 持续更新, 这里只检查真值是否收到
        if self.gazebo_index is None:
            rospy.logwarn_throttle(
                5.0, "gazebo 模型 '%s' 尚未出现在 /gazebo/model_states 中"
                % self.gazebo_model_name)

    def draw_button_clicked(self):
        # 处理Draw按钮点击事件
        self.create_ros_services()
        self.position = Point32()
        group_number = self.group_entry.get()
        if group_number.isdigit() and 1 <= int(group_number) <= 100:
            self.group_number = f"D{int(group_number):02}"
            self.group_error_label.config(text="")
            self.end_label.config(text="Start!", foreground="red")
            self.end = 0
            # 先起表再申请服务: rosservice call 是阻塞的, nav_start 响应慢的
            # 那一两秒如果放在起表之前, 记录的成绩会少掉这段时间
            if not self.small_timer_started:
                self.start_small_timer()
            self.start_big_timer()
            os.system('rosservice call /nav_start "data: 1"')
            rospy.loginfo("向/nav_start申请服务")
            self.flow()
        else:
            self.group_error_label.config(text="Invalid group number")

    def stop_button_clicked(self):
        # 处理Stop按钮点击事件 (不再写 99:99:99, 直接记录表上显示的当前时间;
        # 同时停掉小计时器, 下一组 Draw 时小计时器从零重来)
        if self.big_timer_started:
            self.stop_big_timer()
            self.stop_small_timer()
        else:
            self.group_error_label.config(text="No group is drawing")

    def start_big_timer(self):
        # 启动大计时器 (用墙钟: 比赛计时不依赖仿真钟。慢机器上 gazebo 的
        # 仿真钟比墙钟慢, 原代码会让大计时器走得比小时计时器慢;
        # 真正开跑循环在 draw_button_clicked 末尾的 flow())
        self.big_timer_started = True
        self.big_timer_start_time = time.time()

    def start_small_timer(self):
        # 启动小计时器
        self.small_timer_started = True
        self.small_timer_start_time = time.time()

    def big_timer_running(self):
        elapsed_time = time.time() - self.big_timer_start_time
        formatted_time = time.strftime(
            "%H:%M:%S", time.gmtime(elapsed_time*60))
        self.big_timer_label.config(text=formatted_time)
        self.root.update()

    def small_timer_running(self):
        elapsed_time = time.time() - self.small_timer_start_time
        formatted_time = time.strftime(
            "%H:%M:%S", time.gmtime(elapsed_time*60))
        self.small_timer_label.config(text=formatted_time)
        self.root.update()
        if elapsed_time > 900:  # 15 minutes
            # if elapsed_time > 10:  # 15 minutes
            print("time out")
            self.conform_the_result()

    def stop_big_timer(self):
        # 停止大计时器
        self.end = -2
        self.result_listbox.insert(
            tk.END, f"{self.group_number}: {self.big_timer_label.cget('text')}")
        self.big_timer_started = False
        self.results.append(
            (self.group_number, self.big_timer_label.cget("text")))
        self.results.sort(key=lambda x: x[1])

    def stop_small_timer(self):
        # 停止小计时器
        self.small_timer_started = False

    def conform_the_result(self):
        self.confirm_error_label.config(text="")
        self.big_timer_started = False
        self.stop_small_timer()
        try:
            file = open(self.file_address + "Result.md", "a+")
            file.seek(0)
            s = file.readlines()
            for num in s:
                try:
                    if str(self.results[0][0]) in str(num):
                        self.confirm_error_label.config(
                            text="The group has already been confirmed")
                        break
                except IndexError:
                    self.confirm_error_label.config(
                        text="No grades have been recorded")
            else:
                try:
                    file.write(
                        f"- {self.results[0][0]}: {self.results[0][1]}\n")
                except IndexError:
                    self.confirm_error_label.config(
                        text="No grades have been recorded")
            file.close()
        except FileNotFoundError:
            self.confirm_error_label.config(text="Result.md not found")
        self.group_number = None
        self.big_timer_label.config(text="00:00:00")
        self.end_label.config(text="")
        self.group_entry.delete(0, tk.END)
        self.result_listbox.delete(0, tk.END)
        self.small_timer_label.config(text="00:00:00")

    def confirm_button_clicked(self):
        # 处理Confirm按钮点击事件
        if not self.group_number:
            self.confirm_error_label.config(text="Invalid group number")
        elif len(self.results) <= 2:
            self.confirm_error_label.config(
                text="The number of grades is insufficient")
        else:
            self.conform_the_result()

    def flow(self):
        while self.small_timer_started:
            self.end_box.publish_Area_rectangle(self.end)
            self.small_timer_running()
            if self.big_timer_started:
                self.big_timer_running()
                self._get_robot_position()
                if self.end_box.is_point_in_area(self.position) == self.end and self.zero_vel:
                    self.stop_big_timer()
                    self.stop_small_timer()
                    rospy.loginfo(f"小车已停止")


if __name__ == "__main__":
    try:
        root = tk.Tk()
        app = AccumTimerApp(root)
    except KeyboardInterrupt:
        exit(0)
