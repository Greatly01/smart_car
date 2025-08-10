#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from robot_transport.srv import TransportRequest, TransportRequestResponse
from actionlib_msgs.msg import GoalStatus

class RobotTransportSystem:
    def __init__(self):
        rospy.init_node('robot_transport_system', log_level=rospy.INFO)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        # 增强版连接检查
        max_retries = 3
        for retry in range(max_retries):
            if self.move_base.wait_for_server(rospy.Duration(5)):
                rospy.loginfo("move_base服务器就绪")
                break
            rospy.logwarn(f"move_base连接失败，重试 {retry+1}/{max_retries}")
        else:
            rospy.logfatal("move_base服务器连接失败，退出系统")
            rospy.signal_shutdown("move_base连接失败")
            
        # 加载导航点和任务配置
        self.predefined_waypoints = self._load_waypoints()
        self.mission_config = self._load_mission_config()
        self._validate_config()
        
        self.is_busy = False
        self.current_waypoint = None
        
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
        
        rospy.loginfo("系统就绪，支持任务: A/B/C/D ")

    def _load_waypoints(self):
        """加载导航点"""
        return [
            {
                "position": {"x": 0.11354756355285645, "y": -1.2117512226104736, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": -0.010460992460267514, "w": 0.9999452823213609}
            },#wait-0
            {
                "position": {"x": 3.3290414810180664, "y": -0.14711785316467285, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": -0.5871862467471105, "w": 0.8094518587482775}   
            },#A-1
            {
                "position": {"x": 1.01398766040802, "y": -4.6056365966796875, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.749818314988978, "w": 0.6616437821872807}
            },#B-2
            {
                "position": {"x": 4.6451616287231445, "y": -2.1572749614715576, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.9999991152671494, "w": 0.0013302123584128366}
            },#C-3
            {
                "position": {"x": 4.537341117858887, "y": -0.09103250503540039, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.20979892744873077, "w": 0.9777445525500831}
            },#D-4
            {
                "position": {"x": 0.18768489360809326, "y": -2.350271224975586, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.007346595208499261, "w": 0.9999730134052831}
            },#camp-5
        ]

    def _load_mission_config(self):
        """加载任务配置"""
        return {
            'A': {
                'waypoints': [1, 5, 0],  # 0点出现1次
                'actions': {
                    0: {'delay': 0, 'message': "任务A进度：任务完成,等待下一次任务指令"},
                    1: {'delay': 3, 'message': "任务A进度：到达战区A,已经完成接收伤员任务"},
                    5: {'delay': 3, 'message': "任务A进度：到达我方营地,已完成放置伤员任务"},
                },
                'special_points': {  
                    0: 1,  # 第1次到达0点触发
                    1: 1,
                    5: 1
                }
            },
            'B': {
                'waypoints': [2,5,0],  
                'actions': {
                    0: {'delay': 0, 'message': "任务B进度：任务完成,等待下一次任务指令"},
                    2: {'delay': 3, 'message': "任务B进度：到达战区B,已经完成接收伤员任务"},
                    5: {'delay': 3, 'message': "任务B进度：到达我方营地,已完成放置伤员任务"},
                },
                'special_points': {
                    0: 1,  
                    2: 1,
                    5: 1
                }
            },
            'C': {
                'waypoints': [3,5,0],
                'actions': {
                    0: {'delay': 0, 'message': "任务C进度：任务完成,等待下一次任务指令"},
                    3: {'delay': 3, 'message': "任务C进度：到达战区C,已经完成接收伤员任务"},
                    5: {'delay': 3, 'message': "任务C进度：到达我方营地,已完成放置伤员任务"},
                },
                'special_points': {
                    0: 1,
                    3: 1,
                    5: 1
                }
            },
            'D': {
                'waypoints': [4,5,0],
                'actions': {
                    0: {'delay': 0, 'message': "任务D进度：任务完成,等待下一次任务指令"},
                    4: {'delay': 3, 'message': "任务D进度：到达战区D,已经完成接收伤员任务"},
                    5: {'delay': 3, 'message': "任务D进度：到达我方营地,已完成放置伤员任务"},
                },
                'special_points': {
                    0: 1,
                    4: 1,
                    5: 1
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
        rospy.loginfo(f"成功加载 {count} 个预设目标点，配置验证通过！")

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
        
        rospy.loginfo(f"开始执行任务 {mission_id}，将按照预设导航点执行任务")
        
        for idx in waypoints:
            if rospy.is_shutdown() or not self.is_busy:
                return False, "任务中断"
                
            goal = self.create_goal(idx)
            if not goal:
                rospy.logerr(f"无效目标点 {idx}，跳过")
                continue
                
            self.current_waypoint = idx
            self.move_base.send_goal(goal)
            # rospy.loginfo(f"[任务{mission_id}] 导航至索引 {idx}")
            rospy.loginfo(f"[任务{mission_id}] 导航执行中……")
            if not self.move_base.wait_for_result(rospy.Duration(180)):
                self.move_base.cancel_goal()
                return False, f"导航超时（索引 {idx}）"
                
            if self.move_base.get_state() != GoalStatus.SUCCEEDED:
                return False, f"导航失败（索引 {idx}）"
                
            visit_counts[idx] += 1
            # rospy.loginfo(f"[任务{mission_id}] 到达索引 {idx}（第{visit_counts[idx]}次）")
            rospy.loginfo(f"[任务{mission_id}] 导航执行中……")
            # 执行动作
            if idx in special_points and visit_counts[idx] == special_points[idx]:
                self._execute_action(idx, actions)
            elif idx in actions and idx not in special_points:
                self._execute_action(idx, actions)
                
        self.current_waypoint = None
        rospy.loginfo(f"[任务{mission_id}] 执行完成")
        return True, f"任务 {mission_id} 执行成功"

    def execute_single_waypoint(self, wp_idx):
        """执行单个导航点（强制导航）"""
        goal = self.create_goal(wp_idx)
        if not goal:
            return False, f"无效导航点索引: {wp_idx}"
            
        self.current_waypoint = wp_idx
        self.move_base.send_goal(goal)
        
        rospy.loginfo(f"[强制导航] 导航至索引 {wp_idx}")
        
        if not self.move_base.wait_for_result(rospy.Duration(180)):
            self.move_base.cancel_goal()
            return False, f"导航超时（索引 {wp_idx}）"
            
        if self.move_base.get_state() != GoalStatus.SUCCEEDED:
            return False, f"导航失败（索引 {wp_idx}）"
            
        self.current_waypoint = None
        rospy.loginfo(f"[强制导航] 成功到达导航点 {wp_idx}")
        return True, f"成功到达导航点 {wp_idx}"

    def _execute_action(self, idx, actions):
        """执行指定点的动作"""
        if idx in actions:
            action = actions[idx]
            rospy.loginfo(f"正在执行任务（以延时3s模拟过程）")
            rospy.sleep(action['delay'])
            rospy.loginfo(action['message'])

    def handle_request(self, req):
        """处理标准任务请求"""
        mission_id = req.target.upper()
        
        if mission_id not in self.mission_config:
            return TransportRequestResponse(success=False, message=f"无效任务ID: {mission_id}")
            
        if self.is_busy:
            return TransportRequestResponse(success=False, message="机器人正忙")
            
        try:
            self.is_busy = True
            rospy.loginfo(f"接收任务请求: {mission_id}")
            
            success, msg = self.execute_mission(mission_id)
            
            return TransportRequestResponse(success=success, message=msg)
            
        except Exception as e:
            rospy.logerr(f"任务处理异常: {str(e)}")
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
                return TransportRequestResponse(success=False, message="机器人正忙")
                
            self.is_busy = True
            rospy.loginfo(f"接收强制导航请求: 索引 {wp_idx}")
            
            success, msg = self.execute_single_waypoint(wp_idx)
            
            return TransportRequestResponse(success=success, message=msg)
            
        except ValueError:
            return TransportRequestResponse(success=False, message="无效索引，需输入数字")
        except Exception as e:
            rospy.logerr(f"强制导航异常: {str(e)}")
            return TransportRequestResponse(success=False, message=f"强制导航失败: {str(e)}")
        finally:
            self.is_busy = False

if __name__ == "__main__":
    try:
        system = RobotTransportSystem()
        rospy.loginfo("智能机器人战场伤员运输系统已启动，支持强制导航测试...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("系统关闭，正在取消所有任务...")
        if system.move_base.is_active():
            system.move_base.cancel_all_goals()
        rospy.loginfo("系统安全退出")




