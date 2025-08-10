#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from transport.srv import TransportRequest, TransportRequestResponse
from actionlib_msgs.msg import GoalStatus

class RobotTransportSystem:
    def __init__(self):
        rospy.init_node('robot_transport_system', log_level=rospy.INFO)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        # 增强版连接检查
        max_retries = 3
        for retry in range(max_retries):
            if self.move_base.wait_for_server(rospy.Duration(5)):
                rospy.loginfo("move_base服务器连接成功，通信链路建立")
                print("✅ 机器人控制系统初始化完成，通信状态正常")
                break
            rospy.logwarn(f"move_base连接失败，正在重试 {retry+1}/{max_retries}，请检查网络连接")
        else:
            rospy.logfatal("move_base服务器连接失败，无法建立导航通信")
            print("❌ 致命错误：导航系统连接失败，系统即将退出")
            rospy.signal_shutdown("move_base连接失败")
            
        # 加载导航点和任务配置
        self.predefined_waypoints = self._load_waypoints()
        self.mission_config = self._load_mission_config()
        self._validate_config()
        
        self.is_busy = False
        self.current_waypoint = None
        self.wounded_count = 0  # 新增：伤员运输计数
        
        # 注册标准任务服务
        self.transport_service = rospy.Service(
            'request_transport', 
            TransportRequest, 
            self.handle_request
        )
        
        # 注册强制导航服务
        self.force_nav_service = rospy.Service(
            'force_navigation', 
            TransportRequest,  # 复用消息类型
            self.handle_force_nav
        )
        
        rospy.loginfo("战场伤员运输系统已启动，进入待命状态")
        print(f"✅ 系统就绪，当前可执行任务: A/B/C/D | 强制导航: force_navigation [0-23]")
        print(f"📊 系统状态：导航点加载完成，共{len(self.predefined_waypoints)}个有效点位")

    def _load_waypoints(self):
        """加载导航点（共24个点，索引0-23）"""
        return [
            {
                "position": {"x": 0.14075696468353271, "y": 0.03458595275878906, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": -0.007748185428395918, "w": 0.9999699823607543}
            },#wait-0 - 待命区
            {
                "position": {"x": 3.2480380535125732, "y": -0.018885135650634766, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.7189128925798332, "w": 0.6951001747104494}
            },#1 - 前沿阵地
            {
                "position": {"x": 3.2266483306884766, "y": 0.7117446660995483, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.6881467937203287, "w": 0.7255714921993776}
            },#2 - 临时掩体
            {
                "position": {"x": 2.6811890602111816, "y": 0.7351350784301758, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.7031206916470938, "w": 0.7110705260223577}   
            },#A-3 - 战区A伤员收集点
            {
                "position": {"x": 1.6052896976470947, "y": 0.7183921337127686, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.7025432869457239, "w": 0.7116410120049984}
            },#4 - 通路A
            {
                "position": {"x": 1.2183520793914795, "y": 0.7120131254196167, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.7024911120127548, "w": 0.7116925161494134}
            },#B-5 - 战区B伤员收集点
            {
                "position": {"x": 0.6420629024505615, "y": 0.704096794128418, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.7026089456409802, "w": 0.7115761867188011}
            },#6 - 通路B
            {
                "position": {"x": 0.7038102149963379, "y": 1.5807292461395264, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.43163929759344016, "w": 0.9020462941407396}
            },#7 - 交叉路口
            {
                "position": {"x": 1.0675121545791626, "y": 1.9005951881408691, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.04846021207362573, "w": 0.9988251137440324}
            },#8 - 临时医疗点
            {
                "position": {"x": 1.49245285987854, "y": 2.157886028289795, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.6601465019232154, "w": 0.7511368690182516}
            },#9 - 通路C
            {
                "position": {"x": 1.5385291576385498, "y": 2.717924118041992, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.6597445855490855, "w": 0.7514899080085276}
            },#10 - 通路D
            {
                "position": {"x": 1.9342098236083984, "y": 3.0994648933410645, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": -0.006609808951464038, "w": 0.9999781549742099}
            },#11 - 后方补给点
            {
                "position": {"x": 2.3583853244781494, "y": 2.9856884479522705, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": -0.40841582273849847, "w": 0.9127959880153043}
            },#12 - 通路E
            {
                "position": {"x": 2.558119297027588, "y": 2.283240795135498, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": -0.006464311068990256, "w": 0.9999791061229246}
            },#13 - 安全通道入口
            {
                "position": {"x": 3.3331570625305176, "y": 2.3621702194213867, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.7100650146481949, "w": 0.7041361196335966}
            },#14 - 安全通道中段
            {
                "position": {"x": 3.3940587043762207, "y": 4.352957248687744, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": -0.999973492613726, "w": 0.007281076150300691}
            },#15 - 战区C前沿
            {
                "position": {"x": 3.0527899265289307, "y": 4.303186416625977, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": -0.7117708844398614, "w": 0.7024117083760048}
            },#16 - 战区C通道
            {
                "position": {"x": 2.669312000274658, "y": 4.2886505126953125, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": -0.7111878089691819, "w": 0.7030020628516067}
            },#C-17 - 战区C伤员收集点
            {
                "position": {"x": 1.516380786895752, "y": 4.295287132263184, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": -0.7114636800910995, "w": 0.7027228699218702}
            },#18 - 战区D前沿
            {
                "position": {"x": 1.0635478496551514, "y": 4.266317844390869, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": -0.7206826496211587, "w": 0.6932651141771279}
            },#D-19 - 战区D伤员收集点
            {
                "position": {"x": 0.44453442096710205, "y": 4.243098735809326, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": -0.9803158718038398, "w": 0.19743553755460885}
            },#20 - 后方通路E
            {
                "position": {"x": 0.18168914318084717, "y": 3.3691325187683105, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": -0.778485022037401, "w": 0.627663182338607}
            },#21 - 医疗帐篷区
            {
                "position": {"x": -0.018909692764282227, "y": 1.6211035251617432, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": -0.718701767473324, "w": 0.6953184661942469}
            },#22 - 手术区入口
            {
                "position": {"x": -0.0005364418029785156, "y": 0.0594935417175293, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": -0.6951123327406948, "w": 0.7189011370639147}
            },#camp-23 - 我方营地/急救中心
        ]

    def _load_mission_config(self):
        """加载任务配置"""
        return {
            'A': {
                'waypoints': [0, 1, 2, 3, 2, 1, 0, 23, 0],  # 0点出现3次
                'actions': {
                    0: {'delay': 0, 'message': "✅ 任务完成,等待下一次任务指令"},
                    3: {'delay': 3, 'message': "✅ 到达战区A,执行接收伤员任务"},
                    23: {'delay': 3, 'message': "✅ 到达我方营地,执行放置伤员任务"},
                },
                'special_points': {  
                    0: 3,  # 第三次到达0点触发
                    3: 1,
                    23: 1
                }
            },
            'B': {
                'waypoints': [0, 1, 2, 3, 4, 5, 4, 3, 2, 1, 0, 23, 0],  # 0点出现3次
                'actions': {
                    0: {'delay': 0, 'message': "✅ 任务完成,等待下一次任务指令"},
                    5: {'delay': 3, 'message': "✅ 到达战区B,执行接收伤员任务"},
                    23: {'delay': 3, 'message': "✅ 到达我方营地,执行放置伤员任务"},
                },
                'special_points': {
                    0: 3,  # 第三次到达0点触发
                    5: 1,
                    23: 1
                }
            },
            'C': {
                'waypoints': [23, 22, 21, 20, 19, 18, 17, 18, 19, 20, 21, 22, 23, 0],
                'actions': {
                    0: {'delay': 0, 'message': "✅ 任务完成,等待下一次任务指令"},
                    17: {'delay': 3, 'message': "✅ 到达战区C,执行接收伤员任务"},
                    23: {'delay': 3, 'message': "✅ 到达我方营地,执行放置伤员任务"},
                },
                'special_points': {
                    0: 1,
                    17: 1,
                    23: 2
                }
            },
            'D': {
                'waypoints': [23, 22, 21, 20, 19, 20, 21, 22, 23, 0],
                'actions': {
                    0: {'delay': 0, 'message': "✅ 任务完成,等待下一次任务指令"},
                    19: {'delay': 3, 'message': "✅ 到达战区D,执行接收伤员任务"},
                    23: {'delay': 3, 'message': "✅ 到达我方营地,执行放置伤员任务"},
                },
                'special_points': {
                    0: 1,
                    19: 1,
                    23: 2
                }
            }
        }

    def _validate_config(self):
        """验证配置有效性"""
        count = len(self.predefined_waypoints)
        for mission_id, config in self.mission_config.items():
            for wp_idx in config['waypoints']:
                if wp_idx >= count:
                    rospy.logfatal(f"任务 {mission_id} 包含无效索引 {wp_idx}（总点数 {count}）")
                    rospy.signal_shutdown("配置索引无效")
        rospy.loginfo(f"成功加载 {count} 个导航点，任务配置验证通过")
        print(f"📋 任务配置检查完成，{len(self.mission_config)}项任务计划可用")

    def create_goal(self, idx):
        """创建导航目标"""
        if idx >= len(self.predefined_waypoints):
            rospy.logerr(f"无效导航点索引: {idx}")
            return None
            
        wp = self.predefined_waypoints[idx]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = wp["position"]["x"]
        goal.target_pose.pose.position.y = wp["position"]["y"]
        goal.target_pose.pose.orientation = Quaternion(**wp["orientation"])
        return goal

    def execute_mission(self, mission_id):
        """执行完整任务"""
        if mission_id not in self.mission_config:
            return False, f"无效任务ID: {mission_id}"
            
        config = self.mission_config[mission_id]
        waypoints = config['waypoints']
        actions = config.get('actions', {})
        special_points = config.get('special_points', {})
        
        visit_counts = {wp: 0 for wp in set(waypoints)}
        mission_desc = {
            'A': '战区A伤员回收任务',
            'B': '战区B伤员回收任务',
            'C': '战区C伤员回收任务',
            'D': '战区D伤员回收任务'
        }
        
        rospy.loginfo(f"开始执行{mission_desc.get(mission_id, '未知')}，导航点序列: {waypoints}")
        print(f"🚀 任务启动：{mission_desc.get(mission_id, '未知任务')}，导航路径已规划")

        for idx in waypoints:
            if rospy.is_shutdown() or not self.is_busy:
                print(f"⚠️ 任务中断：接收到停止指令")
                return False, "任务中断"
                
            goal = self.create_goal(idx)
            if not goal:
                rospy.logerr(f"无效目标点 {idx}，跳过")
                print(f"⚠️ 导航异常：目标点{idx}无效，跳过该点位")
                continue
                
            self.current_waypoint = idx
            self.move_base.send_goal(goal)
            
            #显示当前导航点的实际位置描述
            waypoint_desc = {
                0: "待命区", 3: "战区A伤员收集点", 5: "战区B伤员收集点",
                17: "战区C伤员收集点", 19: "战区D伤员收集点", 23: "我方营地/急救中心"
            }
            wp_desc = waypoint_desc.get(idx, f"导航点{idx}")
            
            rospy.loginfo(f"[任务{mission_id}] 导航至{wp_desc}（索引 {idx}）")
            print(f"🚗 正在前往{wp_desc}（索引{idx}），当前任务进度: {waypoints.index(idx)+1}/{len(waypoints)}")
            
            if not self.move_base.wait_for_result(rospy.Duration(180)):
                self.move_base.cancel_goal()
                print(f"⚠️ 导航超时：无法到达{wp_desc}（索引{idx}）")
                return False, f"导航超时（索引 {idx}）"
                
            if self.move_base.get_state() != GoalStatus.SUCCEEDED:
                print(f"⚠️ 导航失败：到达{wp_desc}（索引{idx}）失败")
                return False, f"导航失败（索引 {idx}）"
                
            visit_counts[idx] += 1
            rospy.loginfo(f"[任务{mission_id}] 到达{wp_desc}（第{visit_counts[idx]}次）")
            print(f"✅ 已到达{wp_desc}（索引{idx}），第{visit_counts[idx]}次访问")
            
            # 执行动作
            if idx in special_points and visit_counts[idx] == special_points[idx]:
                self._execute_action(idx, actions, mission_id)
            elif idx in actions and idx not in special_points:
                self._execute_action(idx, actions, mission_id)
                
        self.current_waypoint = None
        self.wounded_count += 1  # 完成一次运输，伤员计数+1
        rospy.loginfo(f"[任务{mission_id}] 执行完成")
        print(f"🎉 {mission_desc.get(mission_id, '未知')}完成！累计运输伤员: {self.wounded_count}人")
        return True, f"任务 {mission_id} 执行成功"

    def execute_single_waypoint(self, wp_idx):
        """执行单个导航点（强制导航）"""
        # 新增：导航点实际位置描述
        waypoint_desc = {
            0: "待命区", 3: "战区A伤员收集点", 5: "战区B伤员收集点",
            17: "战区C伤员收集点", 19: "战区D伤员收集点", 23: "我方营地/急救中心"
        }
        wp_desc = waypoint_desc.get(wp_idx, f"导航点{wp_idx}")
        
        goal = self.create_goal(wp_idx)
        if not goal:
            print(f"⚠️ 强制导航失败：目标点{wp_idx}无效")
            return False, f"无效导航点索引: {wp_idx}"
            
        self.current_waypoint = wp_idx
        self.move_base.send_goal(goal)
        
        rospy.loginfo(f"[强制导航] 导航至{wp_desc}（索引 {wp_idx}）")
        print(f"🚨 强制导航：正在前往{wp_desc}（索引{wp_idx}）")
        
        if not self.move_base.wait_for_result(rospy.Duration(180)):
            self.move_base.cancel_goal()
            print(f"⚠️ 强制导航超时：无法到达{wp_desc}（索引{wp_idx}）")
            return False, f"导航超时（索引 {wp_idx}）"
            
        if self.move_base.get_state() != GoalStatus.SUCCEEDED:
            print(f"⚠️ 强制导航失败：到达{wp_desc}（索引{wp_idx}）失败")
            return False, f"导航失败（索引 {wp_idx}）"
            
        self.current_waypoint = None
        rospy.loginfo(f"[强制导航] 成功到达{wp_desc}")
        print(f"✅ 强制导航完成：已到达{wp_desc}（索引{wp_idx}）")
        return True, f"成功到达导航点 {wp_idx}"

    def _execute_action(self, idx, actions, mission_id=None):
        """执行指定点的动作"""
        if idx in actions:
            action = actions[idx]
            rospy.loginfo(f"在点 {idx} 执行任务")
            
            # 根据不同点位添加具体操作描述
            if idx in [3, 5, 17, 19]:  # 战区伤员收集点
                print(f"🏥 在{idx}号点（战区）执行伤员接收操作，预计耗时{action['delay']}秒")
            elif idx == 23:  # 营地
                print(f"🏥 在{idx}号点（我方营地）执行伤员放置操作，预计耗时{action['delay']}秒")
            
            rospy.sleep(action['delay'])
            print(action['message'])
            
            # 模拟伤员接收/放置操作
            if "到达战区" in action['message']:
                print(f"💹 伤员状态：已固定伤员，生命体征监测中，准备转移")
            elif "到达我方营地" in action['message']:
                print(f"💹 伤员状态：已移交医疗人员，设备消毒中")

    def handle_request(self, req):
        """处理标准任务请求"""
        mission_id = req.target.upper()
        
        if mission_id not in self.mission_config:
            return TransportRequestResponse(success=False, message=f"无效任务ID: {mission_id}")
            
        if self.is_busy:
            return TransportRequestResponse(success=False, message="机器人正忙，当前任务进行中")
            
        try:
            self.is_busy = True
            rospy.loginfo(f"接收任务请求: {mission_id}")
            print(f"📢 收到任务指令：执行{mission_id}号任务，机器人准备出发")
            
            success, msg = self.execute_mission(mission_id)
            
            return TransportRequestResponse(success=success, message=msg)
            
        except Exception as e:
            rospy.logerr(f"任务处理异常: {str(e)}")
            print(f"❌ 任务处理错误：{str(e)}")
            return TransportRequestResponse(success=False, message=f"内部错误: {str(e)}")
        finally:
            self.is_busy = False

    def handle_force_nav(self, req):
        """处理强制导航请求"""
        try:
            wp_idx = int(req.target)  # 将请求的target作为索引
            
            if wp_idx < 0 or wp_idx >= len(self.predefined_waypoints):
                return TransportRequestResponse(success=False, message=f"索引超出范围 [0-{len(self.predefined_waypoints)-1}]")
                
            if self.is_busy:
                return TransportRequestResponse(success=False, message="机器人正忙，当前任务进行中")
                
            self.is_busy = True
            rospy.loginfo(f"接收强制导航请求: 索引 {wp_idx}")
            print(f"📢 收到强制导航指令：前往索引{wp_idx}，机器人立即执行")
            
            success, msg = self.execute_single_waypoint(wp_idx)
            
            return TransportRequestResponse(success=success, message=msg)
            
        except ValueError:
            return TransportRequestResponse(success=False, message="无效索引，需输入数字")
        except Exception as e:
            rospy.logerr(f"强制导航异常: {str(e)}")
            print(f"❌ 强制导航错误：{str(e)}")
            return TransportRequestResponse(success=False, message=f"强制导航失败: {str(e)}")
        finally:
            self.is_busy = False

if __name__ == "__main__":
    try:
        system = RobotTransportSystem()
        rospy.loginfo("机器人战场运输系统已启动，支持强制导航测试...")
        print("===== 战场伤员运输系统启动 =====")
        print("✅ 系统状态：正常运行")
        # print("📌 操作提示：可通过request_transport服务请求A/B/C/D任务")
        # print("📌 操作提示：可通过force_navigation服务强制导航至指定点位")
        rospy.spin()
    except rospy.ROSInterruptException:
        print("===== 系统关闭 =====")
        rospy.loginfo("系统关闭，正在取消所有任务...")
        if system.move_base.is_active():
            system.move_base.cancel_all_goals()
            print("✅ 已取消所有导航任务")
        print(f"👋 系统安全退出，累计完成{system.wounded_count}次伤员运输任务")
        rospy.loginfo("系统安全退出")



















# #!/usr/bin/env python3
# import rospy
# import actionlib
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from geometry_msgs.msg import Quaternion
# from transport.srv import TransportRequest, TransportRequestResponse
# from actionlib_msgs.msg import GoalStatus

# class RobotTransportSystem:
#     def __init__(self):
#         rospy.init_node('robot_transport_system', log_level=rospy.INFO)
#         self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
#         # 增强版连接检查
#         max_retries = 3
#         for retry in range(max_retries):
#             if self.move_base.wait_for_server(rospy.Duration(5)):
#                 rospy.loginfo("move_base服务器连接成功")
#                 break
#             rospy.logwarn(f"move_base连接失败，重试 {retry+1}/{max_retries}")
#         else:
#             rospy.logfatal("move_base服务器连接失败，退出系统")
#             rospy.signal_shutdown("move_base连接失败")
            
#         # 加载导航点和任务配置
#         self.predefined_waypoints = self._load_waypoints()
#         self.mission_config = self._load_mission_config()
#         self._validate_config()
        
#         self.is_busy = False
#         self.current_waypoint = None
        
#         # 注册标准任务服务
#         self.transport_service = rospy.Service(
#             'request_transport', 
#             TransportRequest, 
#             self.handle_request
#         )
        
#         # 注册强制导航服务
#         self.force_nav_service = rospy.Service(
#             'force_navigation', 
#             TransportRequest,  # 复用消息类型
#             self.handle_force_nav
#         )
        
#         rospy.loginfo("系统就绪，支持任务: A/B/C/D | 强制导航: force_navigation [0-23]")

#     def _load_waypoints(self):
#         """加载导航点（共24个点，索引0-23）"""
#         return [
#             {
#                 "position": {"x": 0.14075696468353271, "y": 0.03458595275878906, "z": 0.0},
#                 "orientation": {"x": 0.0, "y": 0.0, "z": -0.007748185428395918, "w": 0.9999699823607543}
#             },#wait-0
#             {
#                 "position": {"x": 3.2480380535125732, "y": -0.018885135650634766, "z": 0.0},
#                 "orientation": {"x": 0.0, "y": 0.0, "z": 0.7189128925798332, "w": 0.6951001747104494}
#             },
#             {
#                 "position": {"x": 3.2266483306884766, "y": 0.7117446660995483, "z": 0.0},
#                 "orientation": {"x": 0.0, "y": 0.0, "z": 0.6881467937203287, "w": 0.7255714921993776}
#             },
#             {
#                 "position": {"x": 2.6811890602111816, "y": 0.7351350784301758, "z": 0.0},
#                 "orientation": {"x": 0.0, "y": 0.0, "z": 0.7031206916470938, "w": 0.7110705260223577}   
#             },#A-3
#             # {
#             #     "position": {"x": 2.278829574584961, "y": 0.7441525459289551, "z": 0.0},
#             #     "orientation": {"x": 0.0, "y": 0.0, "z": 0.7028102107905597, "w": 0.7113774016712431}   
#             # },
#             {
#                 "position": {"x": 1.6052896976470947, "y": 0.7183921337127686, "z": 0.0},
#                 "orientation": {"x": 0.0, "y": 0.0, "z": 0.7025432869457239, "w": 0.7116410120049984}
#             },
#             {
#                 "position": {"x": 1.2183520793914795, "y": 0.7120131254196167, "z": 0.0},
#                 "orientation": {"x": 0.0, "y": 0.0, "z": 0.7024911120127548, "w": 0.7116925161494134}
#             },#B-5
#             {
#                 "position": {"x": 0.6420629024505615, "y": 0.704096794128418, "z": 0.0},
#                 "orientation": {"x": 0.0, "y": 0.0, "z": 0.7026089456409802, "w": 0.7115761867188011}
#             },
#             {
#                 "position": {"x": 0.7038102149963379, "y": 1.5807292461395264, "z": 0.0},
#                 "orientation": {"x": 0.0, "y": 0.0, "z": 0.43163929759344016, "w": 0.9020462941407396}
#             },
#             {
#                 "position": {"x": 1.0675121545791626, "y": 1.9005951881408691, "z": 0.0},
#                 "orientation": {"x": 0.0, "y": 0.0, "z": 0.04846021207362573, "w": 0.9988251137440324}
#             },
#             {
#                 "position": {"x": 1.49245285987854, "y": 2.157886028289795, "z": 0.0},
#                 "orientation": {"x": 0.0, "y": 0.0, "z": 0.6601465019232154, "w": 0.7511368690182516}
#             },
#             {
#                 "position": {"x": 1.5385291576385498, "y": 2.717924118041992, "z": 0.0},
#                 "orientation": {"x": 0.0, "y": 0.0, "z": 0.6597445855490855, "w": 0.7514899080085276}
#             },#10
#             {
#                 "position": {"x": 1.9342098236083984, "y": 3.0994648933410645, "z": 0.0},
#                 "orientation": {"x": 0.0, "y": 0.0, "z": -0.006609808951464038, "w": 0.9999781549742099}
#             },
#             {
#                 "position": {"x": 2.3583853244781494, "y": 2.9856884479522705, "z": 0.0},
#                 "orientation": {"x": 0.0, "y": 0.0, "z": -0.40841582273849847, "w": 0.9127959880153043}
#             },
#             {
#                 "position": {"x": 2.558119297027588, "y": 2.283240795135498, "z": 0.0},
#                 "orientation": {"x": 0.0, "y": 0.0, "z": -0.006464311068990256, "w": 0.9999791061229246}
#             },
#             {
#                 "position": {"x": 3.3331570625305176, "y": 2.3621702194213867, "z": 0.0},
#                 "orientation": {"x": 0.0, "y": 0.0, "z": 0.7100650146481949, "w": 0.7041361196335966}
#             },
#             {
#                 "position": {"x": 3.3940587043762207, "y": 4.352957248687744, "z": 0.0},
#                 "orientation": {"x": 0.0, "y": 0.0, "z": -0.999973492613726, "w": 0.007281076150300691}
#             },
#             {
#                 "position": {"x": 3.0527899265289307, "y": 4.303186416625977, "z": 0.0},
#                 "orientation": {"x": 0.0, "y": 0.0, "z": -0.7117708844398614, "w": 0.7024117083760048}
#             },
#             {
#                 "position": {"x": 2.669312000274658, "y": 4.2886505126953125, "z": 0.0},
#                 "orientation": {"x": 0.0, "y": 0.0, "z": -0.7111878089691819, "w": 0.7030020628516067}
#             },#C-17
#             {
#                 "position": {"x": 1.516380786895752, "y": 4.295287132263184, "z": 0.0},
#                 "orientation": {"x": 0.0, "y": 0.0, "z": -0.7114636800910995, "w": 0.7027228699218702}
#             },
#             {
#                 "position": {"x": 1.0635478496551514, "y": 4.266317844390869, "z": 0.0},
#                 "orientation": {"x": 0.0, "y": 0.0, "z": -0.7206826496211587, "w": 0.6932651141771279}
#             },#D-19
#             {
#                 "position": {"x": 0.44453442096710205, "y": 4.243098735809326, "z": 0.0},
#                 "orientation": {"x": 0.0, "y": 0.0, "z": -0.9803158718038398, "w": 0.19743553755460885}
#             },
#             {
#                 "position": {"x": 0.18168914318084717, "y": 3.3691325187683105, "z": 0.0},
#                 "orientation": {"x": 0.0, "y": 0.0, "z": -0.778485022037401, "w": 0.627663182338607}
#             },
#             {
#                 "position": {"x": -0.018909692764282227, "y": 1.6211035251617432, "z": 0.0},
#                 "orientation": {"x": 0.0, "y": 0.0, "z": -0.718701767473324, "w": 0.6953184661942469}
#             },
#             {
#                 "position": {"x": -0.0005364418029785156, "y": 0.0594935417175293, "z": 0.0},
#                 "orientation": {"x": 0.0, "y": 0.0, "z": -0.6951123327406948, "w": 0.7189011370639147}
#             },#camp-23
#         ]

#     def _load_mission_config(self):
#         """加载任务配置"""
#         return {
#             'A': {
#                 'waypoints': [0, 1, 2, 3, 2, 1, 0, 23, 0],  # 0点出现3次
#                 'actions': {
#                     0: {'delay': 0, 'message': "✅ 任务完成,等待下一次任务指令"},
#                     3: {'delay': 3, 'message': "✅ 到达战区A,执行接收伤员任务"},
#                     23: {'delay': 3, 'message': "✅ 到达我方营地,执行放置伤员任务"},
#                 },
#                 'special_points': {  
#                     0: 3,  # 第三次到达0点触发
#                     3: 1,
#                     23: 1
#                 }
#             },
#             'B': {
#                 'waypoints': [0, 1, 2, 3, 4, 5, 4, 3, 2, 1, 0, 23, 0],  # 0点出现3次
#                 'actions': {
#                     0: {'delay': 0, 'message': "✅ 任务完成,等待下一次任务指令"},
#                     5: {'delay': 3, 'message': "✅ 到达战区B,执行接收伤员任务"},
#                     23: {'delay': 3, 'message': "✅ 到达我方营地,执行放置伤员任务"},
#                 },
#                 'special_points': {
#                     0: 3,  # 第三次到达0点触发
#                     5: 1,
#                     23: 1
#                 }
#             },
#             'C': {
#                 'waypoints': [23, 22, 21, 20, 19, 18, 17, 18, 19, 20, 21, 22, 23, 0],
#                 'actions': {
#                     0: {'delay': 0, 'message': "✅ 任务完成,等待下一次任务指令"},
#                     17: {'delay': 3, 'message': "✅ 到达战区C,执行接收伤员任务"},
#                     23: {'delay': 3, 'message': "✅ 到达我方营地,执行放置伤员任务"},
#                 },
#                 'special_points': {
#                     0: 1,
#                     17: 1,
#                     23: 2
#                 }
#             },
#             'D': {
#                 'waypoints': [23, 22, 21, 20, 19, 20, 21, 22, 23, 0],
#                 'actions': {
#                     0: {'delay': 0, 'message': "✅ 任务完成,等待下一次任务指令"},
#                     19: {'delay': 3, 'message': "✅ 到达战区D,执行接收伤员任务"},
#                     23: {'delay': 3, 'message': "✅ 到达我方营地,执行放置伤员任务"},
#                 },
#                 'special_points': {
#                     0: 1,
#                     19: 1,
#                     23: 2
#                 }
#             }
#         }

#     def _validate_config(self):
#         """验证配置有效性"""
#         count = len(self.predefined_waypoints)
#         for mission_id, config in self.mission_config.items():
#             for wp_idx in config['waypoints']:
#                 if wp_idx >= count:
#                     rospy.logfatal(f"任务 {mission_id} 包含无效索引 {wp_idx}（总点数 {count}）")
#                     rospy.signal_shutdown("配置索引无效")
#         rospy.loginfo(f"成功加载 {count} 个导航点，任务配置验证通过")

#     def create_goal(self, idx):
#         """创建导航目标"""
#         if idx >= len(self.predefined_waypoints):
#             rospy.logerr(f"无效导航点索引: {idx}")
#             return None
            
#         wp = self.predefined_waypoints[idx]
#         goal = MoveBaseGoal()
#         goal.target_pose.header.frame_id = "map"
#         goal.target_pose.header.stamp = rospy.Time.now()
#         goal.target_pose.pose.position.x = wp["position"]["x"]
#         goal.target_pose.pose.position.y = wp["position"]["y"]
#         goal.target_pose.pose.orientation = Quaternion(**wp["orientation"])
#         return goal

#     def execute_mission(self, mission_id):
#         """执行完整任务"""
#         if mission_id not in self.mission_config:
#             return False, f"无效任务ID: {mission_id}"
            
#         config = self.mission_config[mission_id]
#         waypoints = config['waypoints']
#         actions = config.get('actions', {})
#         special_points = config.get('special_points', {})
        
#         visit_counts = {wp: 0 for wp in set(waypoints)}
        
#         rospy.loginfo(f"开始执行任务 {mission_id}，导航点序列: {waypoints}")
        
#         for idx in waypoints:
#             if rospy.is_shutdown() or not self.is_busy:
#                 return False, "任务中断"
                
#             goal = self.create_goal(idx)
#             if not goal:
#                 rospy.logerr(f"无效目标点 {idx}，跳过")
#                 continue
                
#             self.current_waypoint = idx
#             self.move_base.send_goal(goal)
#             rospy.loginfo(f"[任务{mission_id}] 导航至索引 {idx}")
#             # rospy.loginfo(f"[任务{mission_id}] 导航执行中……")
#             if not self.move_base.wait_for_result(rospy.Duration(180)):
#                 self.move_base.cancel_goal()
#                 return False, f"导航超时（索引 {idx}）"
                
#             if self.move_base.get_state() != GoalStatus.SUCCEEDED:
#                 return False, f"导航失败（索引 {idx}）"
                
#             visit_counts[idx] += 1
#             rospy.loginfo(f"[任务{mission_id}] 到达索引 {idx}（第{visit_counts[idx]}次）")
#             # rospy.loginfo(f"[任务{mission_id}] 导航执行中……")
#             # 执行动作
#             if idx in special_points and visit_counts[idx] == special_points[idx]:
#                 self._execute_action(idx, actions)
#             elif idx in actions and idx not in special_points:
#                 self._execute_action(idx, actions)
                
#         self.current_waypoint = None
#         rospy.loginfo(f"[任务{mission_id}] 执行完成")
#         return True, f"任务 {mission_id} 执行成功"

#     def execute_single_waypoint(self, wp_idx):
#         """执行单个导航点（强制导航）"""
#         goal = self.create_goal(wp_idx)
#         if not goal:
#             return False, f"无效导航点索引: {wp_idx}"
            
#         self.current_waypoint = wp_idx
#         self.move_base.send_goal(goal)
        
#         rospy.loginfo(f"[强制导航] 导航至索引 {wp_idx}")
        
#         if not self.move_base.wait_for_result(rospy.Duration(180)):
#             self.move_base.cancel_goal()
#             return False, f"导航超时（索引 {wp_idx}）"
            
#         if self.move_base.get_state() != GoalStatus.SUCCEEDED:
#             return False, f"导航失败（索引 {wp_idx}）"
            
#         self.current_waypoint = None
#         rospy.loginfo(f"[强制导航] 成功到达导航点 {wp_idx}")
#         return True, f"成功到达导航点 {wp_idx}"

#     def _execute_action(self, idx, actions):
#         """执行指定点的动作"""
#         if idx in actions:
#             action = actions[idx]
#             rospy.loginfo(f"在点 {idx} 执行任务")
#             rospy.sleep(action['delay'])
#             print(action['message'])

#     def handle_request(self, req):
#         """处理标准任务请求"""
#         mission_id = req.target.upper()
        
#         if mission_id not in self.mission_config:
#             return TransportRequestResponse(success=False, message=f"无效任务ID: {mission_id}")
            
#         if self.is_busy:
#             return TransportRequestResponse(success=False, message="机器人正忙")
            
#         try:
#             self.is_busy = True
#             rospy.loginfo(f"接收任务请求: {mission_id}")
            
#             success, msg = self.execute_mission(mission_id)
            
#             return TransportRequestResponse(success=success, message=msg)
            
#         except Exception as e:
#             rospy.logerr(f"任务处理异常: {str(e)}")
#             return TransportRequestResponse(success=False, message=f"内部错误: {str(e)}")
#         finally:
#             self.is_busy = False

#     def handle_force_nav(self, req):
#         """处理强制导航请求"""
#         try:
#             wp_idx = int(req.target)  # 将请求的target作为索引
            
#             if wp_idx < 0 or wp_idx >= len(self.predefined_waypoints):
#                 return TransportRequestResponse(success=False, message=f"索引超出范围 [0-{len(self.predefined_waypoints)-1}]")
                
#             if self.is_busy:
#                 return TransportRequestResponse(success=False, message="机器人正忙")
                
#             self.is_busy = True
#             rospy.loginfo(f"接收强制导航请求: 索引 {wp_idx}")
            
#             success, msg = self.execute_single_waypoint(wp_idx)
            
#             return TransportRequestResponse(success=success, message=msg)
            
#         except ValueError:
#             return TransportRequestResponse(success=False, message="无效索引，需输入数字")
#         except Exception as e:
#             rospy.logerr(f"强制导航异常: {str(e)}")
#             return TransportRequestResponse(success=False, message=f"强制导航失败: {str(e)}")
#         finally:
#             self.is_busy = False

# if __name__ == "__main__":
#     try:
#         system = RobotTransportSystem()
#         rospy.loginfo("机器人战场运输系统已启动，支持强制导航测试...")
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         rospy.loginfo("系统关闭，正在取消所有任务...")
#         if system.move_base.is_active():
#             system.move_base.cancel_all_goals()
#         rospy.loginfo("系统安全退出")











# #!/usr/bin/env python3
# import rospy
# import actionlib
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from geometry_msgs.msg import Quaternion
# from robot_transport.srv import TransportRequest, TransportRequestResponse
# from actionlib_msgs.msg import GoalStatus

# class RobotTransportSystem:
#     def __init__(self):
#         rospy.init_node('robot_transport_system', log_level=rospy.INFO)
#         self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
#         # 增强版连接检查
#         max_retries = 3
#         for retry in range(max_retries):
#             if self.move_base.wait_for_server(rospy.Duration(5)):
#                 rospy.loginfo("move_base服务器连接成功")
#                 break
#             rospy.logwarn(f"move_base连接失败，重试 {retry+1}/{max_retries}")
#         else:
#             rospy.logfatal("move_base服务器连接失败，退出系统")
#             rospy.signal_shutdown("move_base连接失败")
            
#         # 导航点配置
#         self.markers = self._load_markers()
#         self.predefined_waypoints = self._load_waypoints()
#         self._validate_waypoints()
        
#         self.is_busy = False
#         self.transport_service = rospy.Service(
#             'request_transport', 
#             TransportRequest, 
#             self.handle_request
#         )
#         rospy.loginfo("系统就绪，支持标记: A/B/C/D/CAMP/WAIT")

#     def _load_markers(self):
#         """加载标记点"""
#         return {
#             'WAIT': [0],
#             'CAMP': [21],
#             'A': [0, 1, 2, 3],
#             'B': [0, 1, 2, 3, 4, 5],
#             'C': [20,19,18,17,16],
#             'D': [20,19,18,17]  # D区终点设为营地
#         }

#     def _load_waypoints(self):
#         """加载导航点"""
#         return [
#         {
#             "position": {"x": 0.14075696468353271, "y": 0.03458595275878906, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.007748185428395918, "w": 0.9999699823607543}
#         },
#         {
#             "position": {"x": 3.2480380535125732, "y": -0.018885135650634766, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.7189128925798332, "w": 0.6951001747104494}
#         },
#         {
#             "position": {"x": 3.2266483306884766, "y": 0.7117446660995483, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.6881467937203287, "w": 0.7255714921993776}
#         },
#         {
#             "position": {"x": 2.6811890602111816, "y": 0.7351350784301758, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.7031206916470938, "w": 0.7110705260223577}   
#         },#A
#         # {
#         #     "position": {"x": 2.278829574584961, "y": 0.7441525459289551, "z": 0.0},
#         #     "orientation": {"x": 0.0, "y": 0.0, "z": 0.7028102107905597, "w": 0.7113774016712431}   
#         # },
#         {
#              "position": {"x": 1.6052896976470947, "y": 0.7183921337127686, "z": 0.0},
#              "orientation": {"x": 0.0, "y": 0.0, "z": 0.7025432869457239, "w": 0.7116410120049984}
#         },
#         {
#             "position": {"x": 1.2183520793914795, "y": 0.7120131254196167, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.7024911120127548, "w": 0.7116925161494134}
#         },#B
#         {
#             "position": {"x": 0.6420629024505615, "y": 0.704096794128418, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.7026089456409802, "w": 0.7115761867188011}
#         },
#         {
#             "position": {"x": 0.7038102149963379, "y": 1.5807292461395264, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.43163929759344016, "w": 0.9020462941407396}
#         },
#         {
#             "position": {"x": 1.0675121545791626, "y": 1.9005951881408691, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.04846021207362573, "w": 0.9988251137440324}
#         },
#         {
#             "position": {"x": 1.49245285987854, "y": 2.157886028289795, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.6601465019232154, "w": 0.7511368690182516}
#         },
#         {
#             "position": {"x": 1.5385291576385498, "y": 2.717924118041992, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.6597445855490855, "w": 0.7514899080085276}
#         },
#         {
#             "position": {"x": 1.9342098236083984, "y": 3.0994648933410645, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.006609808951464038, "w": 0.9999781549742099}
#         },
#         {
#             "position": {"x": 2.3583853244781494, "y": 2.9856884479522705, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.40841582273849847, "w": 0.9127959880153043}
#         },
#         {
#             "position": {"x": 2.558119297027588, "y": 2.283240795135498, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.006464311068990256, "w": 0.9999791061229246}
#         },
#         {
#             "position": {"x": 3.3331570625305176, "y": 2.3621702194213867, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.7100650146481949, "w": 0.7041361196335966}
#         },
#         {
#             "position": {"x": 3.3940587043762207, "y": 4.352957248687744, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.999973492613726, "w": 0.007281076150300691}
#         },
#         {
#             "position": {"x": 3.0527899265289307, "y": 4.303186416625977, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.7117708844398614, "w": 0.7024117083760048}
#         },
#         {
#             "position": {"x": 2.669312000274658, "y": 4.2886505126953125, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.7111878089691819, "w": 0.7030020628516067}
#         },#C
#         {
#             "position": {"x": 1.516380786895752, "y": 4.295287132263184, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.7114636800910995, "w": 0.7027228699218702}
#         },
#         {
#             "position": {"x": 1.0635478496551514, "y": 4.266317844390869, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.7206826496211587, "w": 0.6932651141771279}
#         },#D
#         {
#             "position": {"x": 0.44453442096710205, "y": 4.243098735809326, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.9803158718038398, "w": 0.19743553755460885}
#         },
#         {
#             "position": {"x": 0.18168914318084717, "y": 3.3691325187683105, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.778485022037401, "w": 0.627663182338607}
#         },
#         {
#             "position": {"x": -0.018909692764282227, "y": 1.6211035251617432, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.718701767473324, "w": 0.6953184661942469}
#         },
#         {
#             "position": {"x": -0.0005364418029785156, "y": 0.0594935417175293, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.6951123327406948, "w": 0.7189011370639147}
#         },#camp


#         ]

#     def _validate_waypoints(self):
#         """验证导航点有效性"""
#         count = len(self.predefined_waypoints)
#         for marker, indices in self.markers.items():
#             for idx in indices:
#                 if idx >= count:
#                     rospy.logfatal(f"标记 {marker} 包含无效索引 {idx}（总点数 {count}）")
#                     rospy.signal_shutdown("导航点索引无效")
#         rospy.loginfo(f"成功加载 {count} 个导航点")

#     def create_goal(self, idx):
#         """创建导航目标"""
#         wp = self.predefined_waypoints[idx]
#         goal = MoveBaseGoal()
#         goal.target_pose.header.frame_id = "map"
#         goal.target_pose.header.stamp = rospy.Time.now()
#         goal.target_pose.pose.position.x = wp["position"]["x"]
#         goal.target_pose.pose.position.y = wp["position"]["y"]
#         goal.target_pose.pose.orientation = Quaternion(**wp["orientation"])
#         return goal

#     def navigate_sequence(self, indices, is_return=False):
#         """按顺序导航多个点并执行对应动作"""
#         for idx in indices:
#             if rospy.is_shutdown() or self.is_busy:
#                 return False, "任务中断"
                
#             goal = self.create_goal(idx)
#             if not goal:
#                 return False, f"无效目标点 {idx}"
                
#             self.move_base.send_goal(goal)
#             rospy.loginfo(f"导航至索引 {idx} ({goal.target_pose.pose.position.x:.2f}, {goal.target_pose.pose.position.y:.2f})")
            
#             if not self.move_base.wait_for_result(rospy.Duration(180)):
#                 self.move_base.cancel_goal()
#                 return False, f"导航超时（索引 {idx}）"
                
#             state = self.move_base.get_state()
#             if state != GoalStatus.SUCCEEDED:
#                 return False, f"导航失败（索引 {idx}，状态码 {state}）"
                
#             rospy.loginfo(f"成功到达索引 {idx}")
            
#             # 到达战区点或营地的操作
#             if idx in self.markers['CAMP']:
#                 rospy.sleep(3)
#                 if is_return:
#                     print("✅ 返回营地停留3秒，已移交伤员")
#                 else:
#                     print("✅ 在营地停留3秒，已放置伤员")
#             elif idx in self.markers['A'] + self.markers['B'] + self.markers['C'] + self.markers['D']:
#                 rospy.sleep(3)
#                 if is_return:
#                     print(f"✅ 返回途经战区{idx}停留3秒，已交接物资")
#                 else:
#                     print(f"✅ 在战区{idx}停留3秒，已接收伤员")
                
#         return True, "导航序列完成"

#     def handle_request(self, req):
#         """处理服务请求"""
#         target = req.target.upper()
#         rate = rospy.Rate(1)
        
#         if target not in self.markers and target not in ['A', 'B', 'C', 'D']:
#             return TransportRequestResponse(success=False, message=f"无效标记: {target}")
            
#         if self.is_busy:
#             return TransportRequestResponse(success=False, message="机器人正忙")
            
#         try:
#             self.is_busy = True
#             rospy.loginfo(f"接收到请求：{target}")
            
#             # 处理战区请求（A/B/C/D）
#             if target in ['A', 'B', 'C', 'D']:
#                 # 前往战区
#                 waypoints_to_warzone = self.markers[target]
#                 success, msg = self.navigate_sequence(waypoints_to_warzone)
#                 if not success:
#                     return TransportRequestResponse(success=False, message=f"前往{target}战区失败: {msg}")
                
#                 # 按特定逻辑返回
#                 if target in ['A', 'B']:
#                     # A/B区按原路返回
#                     waypoints_back = waypoints_to_warzone[::-1]
#                 else:
#                     # C/D区按扩展路径返回
#                     current_idx = waypoints_to_warzone[-1]  # 当前位置为战区最后一个点
#                     waypoints_back = list(range(current_idx, 21+1))[::-1]  # 生成[current_idx, 21]的逆序列
                
#                 success, msg = self.navigate_sequence(waypoints_back, is_return=True)
#                 if not success:
#                     return TransportRequestResponse(success=False, message=f"返回失败: {msg}")
                
#                 # 最终返回等待点
#                 if target in ['C', 'D']:  # 从营地返回等待点
#                     success, msg = self.navigate_sequence(self.markers['WAIT'])
#                     if not success:
#                         return TransportRequestResponse(success=False, message=f"返回等待点失败: {msg}")
                    
#                     print("✅ 已返回等待位置，等待新指令")
                
#                 return TransportRequestResponse(success=True, message=f"{target}战区任务完成")
                
#             # 处理单一位置请求（CAMP/WAIT）
#             else:
#                 success, msg = self.navigate_sequence(self.markers[target])
#                 return TransportRequestResponse(success=success, message=msg)
                
#         except Exception as e:
#             rospy.logerr(f"请求处理失败: {str(e)}")
#             return TransportRequestResponse(success=False, message=f"内部错误: {str(e)}")
#         finally:
#             self.is_busy = False
#             rate.sleep()

# if __name__ == "__main__":
#     try:
#         system = RobotTransportSystem()
#         rospy.loginfo("机器人运输系统已启动")
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         rospy.loginfo("系统关闭，正在取消任务...")
#         if system.move_base.is_active():
#             system.move_base.cancel_all_goals()
#         rospy.loginfo("系统安全退出")











# #!/usr/bin/env python3
# import rospy
# import actionlib
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from geometry_msgs.msg import Quaternion
# from robot_transport.srv import TransportRequest, TransportRequestResponse
# from actionlib_msgs.msg import GoalStatus

# class RobotTransportSystem:
#     def __init__(self):
#         rospy.init_node('robot_transport_system', log_level=rospy.INFO)
#         self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
#         # 增强版连接检查，包含重试机制
#         max_retries = 3
#         for retry in range(max_retries):
#             if self.move_base.wait_for_server(rospy.Duration(5)):
#                 rospy.loginfo("move_base服务器连接成功")
#                 break
#             rospy.logwarn(f"move_base连接失败，重试 {retry+1}/{max_retries}")
#         else:
#             rospy.logfatal("move_base服务器连接失败，退出系统")
#             rospy.signal_shutdown("move_base连接失败")
            
#         # 导航点管理（增加安全检查）
#         self.markers = self._load_markers()
#         self.predefined_waypoints = self._load_waypoints()
#         self._validate_waypoints()
        
#         self.is_busy = False
#         self.transport_service = rospy.Service(
#             'request_transport', 
#             TransportRequest, 
#             self.handle_request
#         )
#         rospy.loginfo("系统初始化完成，等待任务指令")

#     def _load_markers(self):
#         """加载标记点（支持从参数服务器动态加载）"""
#         return {
#             'WAIT': [0],
#             'CAMP': [23],
#             'A': [0, 1, 2, 3],
#             'B': [0, 1, 2, 3, 4,5],
#             'C': [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,16,17],
#             'D': [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17,18,19]
#         }

#     def _load_waypoints(self):
#         """加载导航点（支持从YAML文件加载）"""
#         return [
#         # 预设导航点
#         {
#             "position": {"x": 0.14075696468353271, "y": 0.03458595275878906, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.007748185428395918, "w": 0.9999699823607543}
#         },
#         {
#             "position": {"x": 3.2480380535125732, "y": -0.018885135650634766, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.7189128925798332, "w": 0.6951001747104494}
#         },
#         {
#             "position": {"x": 3.2266483306884766, "y": 0.7117446660995483, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.6881467937203287, "w": 0.7255714921993776}
#         },
#         {
#             "position": {"x": 2.6811890602111816, "y": 0.7351350784301758, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.7031206916470938, "w": 0.7110705260223577}   
#         },#A
#         # {
#         #     "position": {"x": 2.278829574584961, "y": 0.7441525459289551, "z": 0.0},
#         #     "orientation": {"x": 0.0, "y": 0.0, "z": 0.7028102107905597, "w": 0.7113774016712431}   
#         # },
#         {
#              "position": {"x": 1.6052896976470947, "y": 0.7183921337127686, "z": 0.0},
#              "orientation": {"x": 0.0, "y": 0.0, "z": 0.7025432869457239, "w": 0.7116410120049984}
#         },
#         {
#             "position": {"x": 1.2183520793914795, "y": 0.7120131254196167, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.7024911120127548, "w": 0.7116925161494134}
#         },#B
#         {
#             "position": {"x": 0.6420629024505615, "y": 0.704096794128418, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.7026089456409802, "w": 0.7115761867188011}
#         },
#         {
#             "position": {"x": 0.7038102149963379, "y": 1.5807292461395264, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.43163929759344016, "w": 0.9020462941407396}
#         },
#         {
#             "position": {"x": 1.0675121545791626, "y": 1.9005951881408691, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.04846021207362573, "w": 0.9988251137440324}
#         },
#         {
#             "position": {"x": 1.49245285987854, "y": 2.157886028289795, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.6601465019232154, "w": 0.7511368690182516}
#         },
#         {
#             "position": {"x": 1.5385291576385498, "y": 2.717924118041992, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.6597445855490855, "w": 0.7514899080085276}
#         },
#         {
#             "position": {"x": 1.9342098236083984, "y": 3.0994648933410645, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.006609808951464038, "w": 0.9999781549742099}
#         },
#         {
#             "position": {"x": 2.3583853244781494, "y": 2.9856884479522705, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.40841582273849847, "w": 0.9127959880153043}
#         },
#         {
#             "position": {"x": 2.558119297027588, "y": 2.283240795135498, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.006464311068990256, "w": 0.9999791061229246}
#         },
#         {
#             "position": {"x": 3.3331570625305176, "y": 2.3621702194213867, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.7100650146481949, "w": 0.7041361196335966}
#         },
#         {
#             "position": {"x": 3.3940587043762207, "y": 4.352957248687744, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.999973492613726, "w": 0.007281076150300691}
#         },
#         {
#             "position": {"x": 3.0527899265289307, "y": 4.303186416625977, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.7117708844398614, "w": 0.7024117083760048}
#         },
#         {
#             "position": {"x": 2.669312000274658, "y": 4.2886505126953125, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.7111878089691819, "w": 0.7030020628516067}
#         },#C
#         {
#             "position": {"x": 1.516380786895752, "y": 4.295287132263184, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.7114636800910995, "w": 0.7027228699218702}
#         },
#         {
#             "position": {"x": 1.0635478496551514, "y": 4.266317844390869, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.7206826496211587, "w": 0.6932651141771279}
#         },#D
#         {
#             "position": {"x": 0.44453442096710205, "y": 4.243098735809326, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.9803158718038398, "w": 0.19743553755460885}
#         },
#         {
#             "position": {"x": 0.18168914318084717, "y": 3.3691325187683105, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.778485022037401, "w": 0.627663182338607}
#         },
#         {
#             "position": {"x": -0.018909692764282227, "y": 1.6211035251617432, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.718701767473324, "w": 0.6953184661942469}
#         },
#         {
#             "position": {"x": -0.0005364418029785156, "y": 0.0594935417175293, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.6951123327406948, "w": 0.7189011370639147}
#         },#camp
#         ]
#     def _validate_waypoints(self):
#         """验证导航点有效性"""
#         count = len(self.predefined_waypoints)
#         for marker, indices in self.markers.items():
#             for idx in indices:
#                 if idx >= count:
#                     rospy.logfatal(f"标记 {marker} 包含无效索引 {idx}（总点数 {count}）")
#                     rospy.signal_shutdown("导航点索引无效")
#         rospy.loginfo(f"成功加载 {count} 个导航点")

#     def create_goal(self, idx):
#         """创建导航目标（带坐标系验证）"""
#         wp = self.predefined_waypoints[idx]
#         goal = MoveBaseGoal()
#         goal.target_pose.header.frame_id = "map"
#         goal.target_pose.header.stamp = rospy.Time.now()
        
#         # 坐标有效性检查
#         if not self._is_valid_pose(wp["position"], wp["orientation"]):
#             rospy.logerr(f"索引 {idx} 坐标或姿态无效")
#             return None
            
#         goal.target_pose.pose.position.x = wp["position"]["x"]
#         goal.target_pose.pose.position.y = wp["position"]["y"]
#         goal.target_pose.pose.orientation = Quaternion(**wp["orientation"])
#         return goal

#     def _is_valid_pose(self, position, orientation):
#         """简单有效性验证（可扩展为距离/姿态范围检查）"""
#         return (
#             abs(position["x"]) < 100 and
#             abs(position["y"]) < 100 and
#             orientation["w"] > 0.5  # 简单验证四元数有效性
#         )

#     def navigate_sequence(self, indices):
#         """带状态跟踪的导航序列"""
#         for idx in indices:
#             goal = self.create_goal(idx)
#             if not goal:
#                 return False, f"索引 {idx} 目标创建失败"
                
#             self.move_base.send_goal(goal)
#             rospy.loginfo(f"[导航任务] 前往索引 {idx} ({goal.target_pose.pose.position.x:.2f}, {goal.target_pose.pose.position.y:.2f})")
            
#             # 带状态反馈的等待
#             status = self.move_base.wait_for_result(rospy.Duration(180))
#             if status:
#                 state = self.move_base.get_state()
#                 if state == GoalStatus.SUCCEEDED:
#                     rospy.loginfo(f"[成功] 到达索引 {idx}")
#                 else:
#                     return False, f"导航失败（状态码: {state}）"
#             else:
#                 return False, "导航超时"
#         return True, "序列执行完成"

#     def execute_full_process(self, war_zone):
#         """带阶段管理的完整任务链"""
#         stages = [
#             ("战区导航", self.markers[war_zone]),
#             ("营地导航", self.markers['CAMP']),
#             ("返回等待点", self.markers['WAIT'])
#         ]
        
#         for stage_name, indices in stages:
#             success, msg = self.navigate_sequence(indices)
#             if not success:
#                 return False, f"{stage_name} 失败: {msg}"
                
#             if stage_name == "战区导航":
#                 rospy.sleep(3)
#                 print(f"✅ 在{war_zone}战区停留3秒，已接收伤员")
#             elif stage_name == "营地导航":
#                 rospy.sleep(3)
#                 print(f"✅ 在营地停留3秒，已放置伤员")
        
#         print("✅ 已返回等待位置，等待新指令")
#         return True, "任务完成"

#     def handle_request(self, req):
#         """带速率控制的请求处理"""
#         target = req.target.upper()
#         rate = rospy.Rate(1)  # 防止CPU占用过高
        
#         if not self._is_valid_target(target):
#             return TransportRequestResponse(success=False, message="无效目标")
            
#         if self.is_busy:
#             return TransportRequestResponse(success=False, message="机器人正忙")
            
#         try:
#             self.is_busy = True
#             rospy.loginfo(f"接收到请求：{target}")
            
#             if target in ['A', 'B', 'C', 'D']:
#                 success, msg = self.execute_full_process(target)
#             else:
#                 success, msg = self.navigate_sequence(self.markers[target])
#                 if success and target == 'CAMP':
#                     print("✅ 在营地停留3秒，已放置伤员")
#                 elif success and target == 'WAIT':
#                     print("✅ 已返回等待位置")
            
#             return TransportRequestResponse(success=success, message=msg)
            
#         except Exception as e:
#             rospy.logerr(f"请求处理失败: {str(e)}")
#             return TransportRequestResponse(success=False, message=f"内部错误: {str(e)}")
#         finally:
#             self.is_busy = False
#             rate.sleep()  # 释放CPU资源

#     def _is_valid_target(self, target):
#         """目标合法性检查（支持正则表达式扩展）"""
#         return target in self.markers or target.upper() in ['A', 'B', 'C', 'D']

# if __name__ == "__main__":
#     try:
#         system = RobotTransportSystem()
#         rospy.loginfo("机器人运输系统已启动")
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         rospy.loginfo("接收到中断信号，正在停止运动...")
#         if system.move_base.is_active():
#             system.move_base.cancel_all_goals()
#         rospy.loginfo("系统安全关闭")















# #!/usr/bin/env python3
# import rospy
# import actionlib
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from geometry_msgs.msg import Quaternion
# from robot_transport.srv import TransportRequest, TransportRequestResponse  # 自定义服务

# class RobotTransportSystem:
#     def __init__(self):
#         rospy.init_node('robot_transport_system')
#         self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
#         self.move_base.wait_for_server(rospy.Duration(5))
#         rospy.loginfo("导航系统已启动")
        
#         # # 定义标记导航点（索引对应predefined_waypoints的顺序）
#         # self.markers = {
#         #     'WAIT': [0],            # 等待位置（索引0）
#         #     'CAMP': [22],           # 营地（索引22，对应predefined_waypoints最后一个点）
#         #     'A': [0, 1, 2, 3],      # 战区A路径：索引0→1→2→3
#         #     'B': [0, 1, 2, 3, 4, 5], # 战区B路径：索引0→1→2→3→4→5
#         #     'C': [0, 1, 2, ..., 16], # 请根据实际长度补全索引（示例省略中间点）
#         #     'D': [0, 1, 2, ..., 18]  # 请根据实际长度补全索引（示例省略中间点）
#         # }
        
#         # # 预设导航点（与用户提供的列表一致，索引从0开始）
#         # self.predefined_waypoints = [
#         #     {"position": {"x": 0.140757, "y": 0.034586, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": -0.007748, "w": 0.999970}},
#         #     {"position": {"x": 3.248038, "y": -0.018885, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.718913, "w": 0.695100}},
#         #     {"position": {"x": 3.261110, "y": 0.680927, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 1.0, "w": 0.0}},
#         #     {"position": {"x": 2.681189, "y": 0.735135, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.703121, "w": 0.711071}},
#         #     # 省略中间导航点...
#         #     {"position": {"x": -0.000536, "y": 0.059494, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": -0.695112, "w": 0.718901}}  # 索引22（CAMP）
#         # ]
#         # 定义标记导航点（格式：标记: [导航点索引列表]）
#         self.markers = {
#             'WAIT': [0],           # 等待位置（原预设第1点）
#             'CAMP': [21],           # 我方营地（原预设第1点，需根据实际调整）
#             'A': [0, 1,2,3],                                        # 战区A（路径点序列）
#             'B': [0, 1,2,3,4],                                      # 战区B（路径点序列）
#             'C': [0, 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15],       # 战区C（路径点序列）
#             'D': [0, 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17]  # 战区D（路径点序列）
#         }
        
#         # 预设导航点
#         self.predefined_waypoints = [
#         {
#             "position": {"x": 0.14075696468353271, "y": 0.03458595275878906, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.007748185428395918, "w": 0.9999699823607543}
#         },
#         {
#             "position": {"x": 3.2480380535125732, "y": -0.018885135650634766, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.7189128925798332, "w": 0.6951001747104494}
#         },
#         {
#             "position": {"x": 3.2611098289489746, "y": 0.6809267997741699, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.9999981779099705, "w": 0.0019089726920353278}
#         },
#         {
#             "position": {"x": 2.6811890602111816, "y": 0.7351350784301758, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.7031206916470938, "w": 0.7110705260223577}   
#         },#A
#         # {
#         #     "position": {"x": 2.278829574584961, "y": 0.7441525459289551, "z": 0.0},
#         #     "orientation": {"x": 0.0, "y": 0.0, "z": 0.7028102107905597, "w": 0.7113774016712431}   
#         # },
#          {
#              "position": {"x": 1.6052896976470947, "y": 0.7183921337127686, "z": 0.0},
#              "orientation": {"x": 0.0, "y": 0.0, "z": 0.7025432869457239, "w": 0.7116410120049984}
#         },
#         {
#             "position": {"x": 1.2183520793914795, "y": 0.7120131254196167, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.7024911120127548, "w": 0.7116925161494134}
#         },#B
#         {
#             "position": {"x": 0.6420629024505615, "y": 0.704096794128418, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.7026089456409802, "w": 0.7115761867188011}
#         },
#         {
#             "position": {"x": 0.7038102149963379, "y": 1.5807292461395264, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.43163929759344016, "w": 0.9020462941407396}
#         },
#         {
#             "position": {"x": 1.0675121545791626, "y": 1.9005951881408691, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.04846021207362573, "w": 0.9988251137440324}
#         },
#         {
#             "position": {"x": 1.49245285987854, "y": 2.157886028289795, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.6601465019232154, "w": 0.7511368690182516}
#         },
#         {
#             "position": {"x": 1.5385291576385498, "y": 2.717924118041992, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.6597445855490855, "w": 0.7514899080085276}
#         },
#         {
#             "position": {"x": 1.9342098236083984, "y": 3.0994648933410645, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.006609808951464038, "w": 0.9999781549742099}
#         },
#         {
#             "position": {"x": 2.3583853244781494, "y": 2.9856884479522705, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.40841582273849847, "w": 0.9127959880153043}
#         },
#         {
#             "position": {"x": 2.558119297027588, "y": 2.283240795135498, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.006464311068990256, "w": 0.9999791061229246}
#         },
#         {
#             "position": {"x": 3.3331570625305176, "y": 2.3621702194213867, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.7100650146481949, "w": 0.7041361196335966}
#         },
#         {
#             "position": {"x": 3.3940587043762207, "y": 4.352957248687744, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.999973492613726, "w": 0.007281076150300691}
#         },
#         {
#             "position": {"x": 3.0527899265289307, "y": 4.303186416625977, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.7117708844398614, "w": 0.7024117083760048}
#         },
#         {
#             "position": {"x": 2.669312000274658, "y": 4.2886505126953125, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.7111878089691819, "w": 0.7030020628516067}
#         },#C
#         {
#             "position": {"x": 1.516380786895752, "y": 4.295287132263184, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.7114636800910995, "w": 0.7027228699218702}
#         },
#         {
#             "position": {"x": 1.0635478496551514, "y": 4.266317844390869, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.7206826496211587, "w": 0.6932651141771279}
#         },#D
#         {
#             "position": {"x": 0.44453442096710205, "y": 4.243098735809326, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.9803158718038398, "w": 0.19743553755460885}
#         },
#         {
#             "position": {"x": 0.18168914318084717, "y": 3.3691325187683105, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.778485022037401, "w": 0.627663182338607}
#         },
#         {
#             "position": {"x": -0.018909692764282227, "y": 1.6211035251617432, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.718701767473324, "w": 0.6953184661942469}
#         },
#         {
#             "position": {"x": -0.0005364418029785156, "y": 0.0594935417175293, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.6951123327406948, "w": 0.7189011370639147}
#         },#camp
#         ]
        
#         self.is_busy = False
#         # 注册自定义服务
#         self.transport_service = rospy.Service(
#             'request_transport', 
#             TransportRequest, 
#             self.handle_request
#         )
#         rospy.loginfo("等待指令：输入战区标记（A/B/C/D）或功能标记（CAMP/WAIT）")

#     def create_goal(self, idx):
#         """根据索引创建导航目标"""
#         wp = self.predefined_waypoints[idx]
#         goal = MoveBaseGoal()
#         goal.target_pose.header.frame_id = "map"
#         goal.target_pose.pose.position.x = wp["position"]["x"]
#         goal.target_pose.pose.position.y = wp["position"]["y"]
#         goal.target_pose.pose.orientation = Quaternion(
#             x=wp["orientation"]["x"],
#             y=wp["orientation"]["y"],
#             z=wp["orientation"]["z"],
#             w=wp["orientation"]["w"]
#         )
#         return goal

#     def navigate_sequence(self, indices):
#         """按顺序导航多个点"""
#         for idx in indices:
#             if rospy.is_shutdown() or self.is_busy:
#                 return False
#             goal = self.create_goal(idx)
#             self.move_base.send_goal(goal)
            
#             if not self.move_base.wait_for_result(rospy.Duration(60)):
#                 self.move_base.cancel_goal()
#                 rospy.logerr(f"导航到索引 {idx} 失败")
#                 return False
#         return True

#     def execute_full_process(self, war_zone):
#         """执行完整任务链：战区→营地→等待位置"""
#         try:
#             # 1. 导航至战区
#             if not self.navigate_sequence(self.markers[war_zone]):
#                 return False, "导航至战区失败"
#             rospy.sleep(3)
#             print(f"✅ 在{war_zone}战区停留3秒，已接收伤员")
            
#             # 2. 导航至营地
#             if not self.navigate_sequence(self.markers['CAMP']):
#                 return False, "导航至营地失败"
#             rospy.sleep(3)
#             print(f"✅ 在营地停留3秒，已放置伤员")
            
#             # 3. 导航至等待位置
#             if not self.navigate_sequence(self.markers['WAIT']):
#                 return False, "导航至等待位置失败"
#             print("✅ 已返回等待位置，等待新指令")
#             return True, "完整任务已完成"
            
#         except Exception as e:
#             rospy.logerr(f"任务链中断: {str(e)}")
#             return False, f"任务链中断: {str(e)}"

#     def handle_request(self, req):
#         """处理服务请求"""
#         if self.is_busy:
#             return TransportRequestResponse(success=False, message="BUSY: 机器人正在执行任务")
            
#         target = req.target.upper()
#         if target not in self.markers:
#             return TransportRequestResponse(success=False, message="ERROR: 无效标记（支持A/B/C/D/CAMP/WAIT）")
            
#         try:
#             self.is_busy = True
#             rospy.loginfo(f"接收到请求：目标 {target}")
            
#             # 处理战区请求（自动执行全流程）
#             if target in ['A', 'B', 'C', 'D']:
#                 success, message = self.execute_full_process(target)
#                 return TransportRequestResponse(success=success, message=message)
                
#             # 处理单一位置请求（CAMP/WAIT）
#             else:
#                 if not self.navigate_sequence(self.markers[target]):
#                     return TransportRequestResponse(success=False, message=f"FAILURE: 导航至 {target} 失败")
                    
#                 if target == 'CAMP':
#                     rospy.sleep(3)
#                     print("✅ 在营地停留3秒，已放置伤员")
#                 elif target == 'WAIT':
#                     print("✅ 已返回等待位置")
#                 return TransportRequestResponse(success=True, message=f"SUCCESS: 已到达 {target}")
                
#         except Exception as e:
#             rospy.logerr(f"处理请求失败: {str(e)}")
#             return TransportRequestResponse(success=False, message=f"ERROR: {str(e)}")
#         finally:
#             self.is_busy = False

# def main():
#     try:
#         rospy.init_node('robot_transport_system')
#         system = RobotTransportSystem()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         rospy.loginfo("系统关闭")

# if __name__ == "__main__":
#     main()











# #!/usr/bin/env python3
# import rospy
# import actionlib
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from geometry_msgs.msg import Quaternion
# from std_srvs.srv import String, StringResponse

# class RobotTransportSystem:
#     def __init__(self):
#         rospy.init_node('robot_transport_system')
#         self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
#         self.move_base.wait_for_server(rospy.Duration(5))
#         rospy.loginfo("导航系统已启动")
        
#         # 定义标记导航点（格式：标记: [导航点索引列表]）
#         self.markers = {
#             'WAIT': [0],           # 等待位置（原预设第1点）
#             'CAMP': [22],           # 我方营地（原预设第1点，需根据实际调整）
#             'A': [0, 1,2,3],                                        # 战区A（路径点序列）
#             'B': [0, 1,2,3,4],                                      # 战区B（路径点序列）
#             'C': [0, 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16],       # 战区C（路径点序列）
#             'D': [0, 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18]  # 战区D（路径点序列）
#         }
        
#         # 预设导航点
#         self.predefined_waypoints = [
#         {
#             "position": {"x": 0.14075696468353271, "y": 0.03458595275878906, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.007748185428395918, "w": 0.9999699823607543}
#         },
#         {
#             "position": {"x": 3.2480380535125732, "y": -0.018885135650634766, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.7189128925798332, "w": 0.6951001747104494}
#         },
#         {
#             "position": {"x": 3.2611098289489746, "y": 0.6809267997741699, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.9999981779099705, "w": 0.0019089726920353278}
#         },
#         {
#             "position": {"x": 2.6811890602111816, "y": 0.7351350784301758, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.7031206916470938, "w": 0.7110705260223577}   
#         },#A
#         # {
#         #     "position": {"x": 2.278829574584961, "y": 0.7441525459289551, "z": 0.0},
#         #     "orientation": {"x": 0.0, "y": 0.0, "z": 0.7028102107905597, "w": 0.7113774016712431}   
#         # },
#         # {
#         #     "position": {"x": 1.6052896976470947, "y": 0.7183921337127686, "z": 0.0},
#         #     "orientation": {"x": 0.0, "y": 0.0, "z": 0.7025432869457239, "w": 0.7116410120049984}
#         # },
#         {
#             "position": {"x": 1.2183520793914795, "y": 0.7120131254196167, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.7024911120127548, "w": 0.7116925161494134}
#         },#B
#         {
#             "position": {"x": 0.6420629024505615, "y": 0.704096794128418, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.7026089456409802, "w": 0.7115761867188011}
#         },
#         {
#             "position": {"x": 0.7038102149963379, "y": 1.5807292461395264, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.43163929759344016, "w": 0.9020462941407396}
#         },
#         {
#             "position": {"x": 1.0675121545791626, "y": 1.9005951881408691, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.04846021207362573, "w": 0.9988251137440324}
#         },
#         {
#             "position": {"x": 1.49245285987854, "y": 2.157886028289795, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.6601465019232154, "w": 0.7511368690182516}
#         },
#         {
#             "position": {"x": 1.5385291576385498, "y": 2.717924118041992, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.6597445855490855, "w": 0.7514899080085276}
#         },
#         {
#             "position": {"x": 1.9342098236083984, "y": 3.0994648933410645, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.006609808951464038, "w": 0.9999781549742099}
#         },
#         {
#             "position": {"x": 2.3583853244781494, "y": 2.9856884479522705, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.40841582273849847, "w": 0.9127959880153043}
#         },
#         {
#             "position": {"x": 2.558119297027588, "y": 2.283240795135498, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.006464311068990256, "w": 0.9999791061229246}
#         },
#         {
#             "position": {"x": 3.3331570625305176, "y": 2.3621702194213867, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": 0.7100650146481949, "w": 0.7041361196335966}
#         },
#         {
#             "position": {"x": 3.3940587043762207, "y": 4.352957248687744, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.999973492613726, "w": 0.007281076150300691}
#         },
#         {
#             "position": {"x": 3.0527899265289307, "y": 4.303186416625977, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.7117708844398614, "w": 0.7024117083760048}
#         },
#         {
#             "position": {"x": 2.669312000274658, "y": 4.2886505126953125, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.7111878089691819, "w": 0.7030020628516067}
#         },#C
#         {
#             "position": {"x": 1.516380786895752, "y": 4.295287132263184, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.7114636800910995, "w": 0.7027228699218702}
#         },
#         {
#             "position": {"x": 1.0635478496551514, "y": 4.266317844390869, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.7206826496211587, "w": 0.6932651141771279}
#         },#D
#         {
#             "position": {"x": 0.44453442096710205, "y": 4.243098735809326, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.9803158718038398, "w": 0.19743553755460885}
#         },
#         {
#             "position": {"x": 0.18168914318084717, "y": 3.3691325187683105, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.778485022037401, "w": 0.627663182338607}
#         },
#         {
#             "position": {"x": -0.018909692764282227, "y": 1.6211035251617432, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.718701767473324, "w": 0.6953184661942469}
#         },
#         {
#             "position": {"x": -0.0005364418029785156, "y": 0.0594935417175293, "z": 0.0},
#             "orientation": {"x": 0.0, "y": 0.0, "z": -0.6951123327406948, "w": 0.7189011370639147}
#         },#camp
#         ]
        
#         self.is_busy = False
#         self.transport_service = rospy.Service('request_transport', String, self.handle_request)
#         rospy.loginfo("等待任务指令：输入战区标记（A/B/C/D）自动执行全流程")

#     def create_goal(self, idx):
#         """根据索引创建导航目标"""
#         wp = self.predefined_waypoints[idx]
#         goal = MoveBaseGoal()
#         goal.target_pose.header.frame_id = "map"
#         goal.target_pose.pose.position.x = wp["position"]["x"]
#         goal.target_pose.pose.position.y = wp["position"]["y"]
#         goal.target_pose.pose.orientation = Quaternion(
#             x=wp["orientation"]["x"],
#             y=wp["orientation"]["y"],
#             z=wp["orientation"]["z"],
#             w=wp["orientation"]["w"]
#         )
#         return goal

#     def navigate_sequence(self, indices):
#         """按顺序导航多个点"""
#         for idx in indices:
#             if rospy.is_shutdown() or self.is_busy:
#                 return False
#             goal = self.create_goal(idx)
#             self.move_base.send_goal(goal)
            
#             if not self.move_base.wait_for_result(rospy.Duration(60)):
#                 self.move_base.cancel_goal()
#                 rospy.logerr(f"导航到索引 {idx} 失败")
#                 return False
#         return True

#     def execute_full_process(self, war_zone):
#         """执行完整任务链：战区→营地→等待位置"""
#         try:
#             # 1. 导航至战区
#             if not self.navigate_sequence(self.markers[war_zone]):
#                 return False
#             rospy.sleep(3)
#             print(f"✅ 在{war_zone}战区停留3秒，已接收伤员")
            
#             # 2. 导航至营地
#             if not self.navigate_sequence(self.markers['CAMP']):
#                 return False
#             rospy.sleep(3)
#             print(f"✅ 在营地停留3秒，已放置伤员")
            
#             # 3. 导航至等待位置
#             if not self.navigate_sequence(self.markers['WAIT']):
#                 return False
#             print("✅ 已返回等待位置，等待新指令")
#             return True
            
#         except Exception as e:
#             rospy.logerr(f"任务链中断: {str(e)}")
#             self.is_busy = False
#             return False

#     def handle_request(self, req):
#         """处理服务请求"""
#         if self.is_busy:
#             return StringResponse("BUSY: 机器人正在执行任务")
            
#         cmd = req.data.upper()
#         if cmd not in ['A', 'B', 'C', 'D']:
#             return StringResponse("ERROR: 请输入有效的战区标记（A/B/C/D）")
            
#         try:
#             self.is_busy = True
#             rospy.loginfo(f"接收到任务：前往{cmd}战区并完成全流程")
            
#             if not self.execute_full_process(cmd):
#                 return StringResponse(f"FAILURE: 任务链执行失败")
                
#             return StringResponse("SUCCESS: 完整任务已完成")
            
#         except Exception as e:
#             rospy.logerr(f"处理请求失败: {str(e)}")
#             return StringResponse(f"ERROR: {str(e)}")
#         finally:
#             self.is_busy = False

# def main():
#     try:
#         system = RobotTransportSystem()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         rospy.loginfo("系统关闭")

# if __name__ == "__main__":
#     main()
