#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion

class AutoPhotoNavigation:
    def __init__(self):
        rospy.init_node('auto_photo_navigation', anonymous=True)
        self.bridge = CvBridge()

        self.take_photo = False
        self.waypoints = []
        self.current_waypoint_index = 0
        self.photo_save_path = "/home/demo/yolov5-master/data/images"  # 根据实际修改

        self.image_sub = rospy.Subscriber('/cam', Image, self.image_callback)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5))
        rospy.loginfo("AutoPhotoNavigation initialized, take_photo ready.")

    def image_callback(self, msg):
        """处理图像消息，触发拍照时保存图片"""
        if self.take_photo:
            try:
                
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                
                image_name = f"photo_{self.current_waypoint_index}.jpg"
                full_path = f"{self.photo_save_path}/{image_name}"  
                # 保存图片
                cv2.imwrite(full_path, cv_image)
                rospy.loginfo(f"成功保存照片：{full_path}")
                self.take_photo = False  # 重置拍照标志
            except Exception as e:
                rospy.logerr(f"保存照片失败：{str(e)}")

    def add_waypoint(self, position, orientation):
        """将用户指定的导航点转换为MoveBaseGoal格式并添加到列表"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        # 填充位置信息
        goal.target_pose.pose.position.x = position["x"]
        goal.target_pose.pose.position.y = position["y"]
        goal.target_pose.pose.position.z = position["z"]
        # 填充姿态信息（四元数）
        goal.target_pose.pose.orientation = Quaternion(
            x=orientation["x"],
            y=orientation["y"],
            z=orientation["z"],
            w=orientation["w"]
        )
        self.waypoints.append(goal)

    def navigate_to_waypoint(self, photo_points):
        """递归导航到指定的导航点列表，到达拍照点时触发拍照"""
       
        if rospy.is_shutdown() or self.current_waypoint_index >= len(self.waypoints):
            return

        current_goal = self.waypoints[self.current_waypoint_index]
        self.move_base.send_goal(current_goal)
        rospy.loginfo(f"正在导航到第 {self.current_waypoint_index} 个目标点")

        if self.move_base.wait_for_result(rospy.Duration(60)):
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("成功到达目标点")
                # 触发拍照（如果当前点是拍照点）
                if self.current_waypoint_index in photo_points:
                    self.take_photo = True
                # 移动到下一个目标点
                self.current_waypoint_index += 1
                self.navigate_to_waypoint(photo_points)  # 递归导航
            else:
                rospy.logerr(f"导航失败，状态码：{state}")
        else:
            self.move_base.cancel_goal()
            rospy.logerr("导航超时，已取消目标")

    def run(self):
        """主流程：获取用户输入、处理导航点、启动导航"""
        # 预设导航点（用户自定义）
        predefined_waypoints = [
            {
                "position": {"x":3.273721694946289, "y": 0.10229675769805908, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.7036439324607633, "w": 0.7105527540662642}
            },
            {
                "position": {"x": 3.2739059181213379, "y": 1.1717101240158081, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.9999863994050228, "w": 0.0052154582711724075}
            },
            {
                "position": {"x": 2.712613582611084, "y": 1.171734094619751, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.7048095761614968, "w": 0.7093965473210673}
            },#识别社区人员点1
            {
                "position": {"x": 2.708164691925049, "y": 1.1628698110580444, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": -0.7025514309847865, "w": 0.7116329719885307}
            },#识别社区人员点2
            {
                "position": {"x": 1.7560567617416382, "y":1.1601985502243042, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.7019568442032663, "w": 0.7122194808317105}
            },
            {
                "position": {"x": 1.7512679767608643, "y": 3.254340400695801, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": -0.999909926731444, "w": 0.013421565628430335}
            },
            {
                "position": {"x": 0.6506192398071289, "y": 3.255295467376709, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": -0.7094742501677804, "w": 0.7047313589935286}
            },
            {
                "position": {"x": 0.6554547691345215, "y": 2.655984001159668, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": -0.9999054648740497, "w": 0.013749956909402972}
            },#识别电动车点3
            {
                "position": {"x": 0.6532302761077881, "y": 0.05102050304412842, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": -0.999940746469389, "w": 0.010885933595291332}
            },
            {
                "position": {"x": 0.10546386241912842, "y": 0.04395115375518799, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": -0.9998868816654268, "w": 0.01504074045347016}
            },
        ]
        # 预设拍照点
        predefined_photo_points = [2,3,7]

 

 
        #开启调试功能，手动输入调试点
        while True:
            mode = input("请选择模式（1: 手动输入; 2: 使用预设）：")
            if mode == '1':
                # 获取用户输入的导航点编号
                valid_waypoints = self._get_valid_indices(
                    predefined_waypoints,
                    "请输入要去的导航点编号（用逗号分隔，如 0,1,2）：",
                    "导航点"
                )

                # 获取用户输入的拍照点编号
                valid_photo_points = self._get_valid_indices(
                    predefined_waypoints,
                    "请输入要拍照的导航点编号（用逗号分隔，如 0,1,2）：",
                    "拍照点"
                )
                break
            elif mode == '2':
                valid_waypoints = list(range(len(predefined_waypoints)))
                valid_photo_points = predefined_photo_points
                break
            else:
                print("输入无效，请输入 1 或 2。")
        
        
        
        # 开启全自动模式，使用预设导航拍照点
        # valid_waypoints = list(range(len(predefined_waypoints)))  # 使用所有导航点
        # valid_photo_points = predefined_photo_points


        
        
        # 添加用户选择的导航点到任务列表
        for idx in valid_waypoints:
            self.add_waypoint(
                predefined_waypoints[idx]["position"],
                predefined_waypoints[idx]["orientation"]
            )

        # 启动导航流程
        self.navigate_to_waypoint(valid_photo_points)

        # 任务完成后自动退出
        if self.current_waypoint_index == len(self.waypoints):
            rospy.loginfo("所有任务已完成，程序自动退出。")
            rospy.signal_shutdown("任务完成")

    def _get_valid_indices(self, point_list, prompt, point_type):
        """辅助函数：获取用户输入的有效索引（处理输入验证）"""
        while True:
            try:
                user_input = input(prompt).strip()
                if not user_input:
                    print(f"错误：未输入{point_type}编号，请重新输入。")
                    continue
                indices = [int(i.strip()) for i in user_input.split(',')]
                valid_indices = [i for i in indices if 0 <= i < len(point_list)]
                if not valid_indices:
                    print(f"错误：{point_type}编号无效（范围0-{len(point_list)-1}），请重新输入。")
                    continue
                return valid_indices
            except ValueError:
                print(f"错误：输入无效，请输入有效的整数{point_type}编号（如0,1,2）。")

if __name__ == "__main__":
    try:
        # 初始化导航与拍照控制类
        navigator = AutoPhotoNavigation()
        # 运行主流程
        navigator.run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        # 捕获Ctrl+C或ROS中断信号
        rospy.loginfo("\n接收到终止信号，正在取消未完成的导航任务...")
        if navigator.move_base.is_active():
            navigator.move_base.cancel_all_goals()  # 取消所有未完成的导航目标
        rospy.loginfo("程序已安全终止，再见！")
