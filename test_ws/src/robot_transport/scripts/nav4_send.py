#!/usr/bin/env python3
import rospy
from robot_transport.srv import TransportRequest, TransportRequestRequest

def send_transport_request(target):
    """发送运输请求到服务端"""
    try:
        # 创建服务代理
        request = rospy.ServiceProxy('request_transport', TransportRequest)
        # 发送请求
        response = request(TransportRequestRequest(target=target))
        # 打印响应
        print(f"服务响应: {response.message}")
        return response.success
        
    except rospy.ServiceException as e:
        print(f"服务调用失败: {e}")
        return False
    except rospy.ROSInterruptException:
        print("请求中断")
        return False

def send_force_navigation(wp_idx):
    """发送强制导航请求到服务端"""
    try:
        # 创建服务代理
        request = rospy.ServiceProxy('force_navigation', TransportRequest)
        # 发送请求（将索引作为target）
        response = request(TransportRequestRequest(target=str(wp_idx)))
        # 打印响应
        print(f"强制导航响应: {response.message}")
        return response.success
        
    except rospy.ServiceException as e:
        print(f"服务调用失败: {e}")
        return False
    except rospy.ROSInterruptException:
        print("请求中断")
        return False

if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node('transport_client', anonymous=True)
    
    print("==== 机器人战场伤员运输系统客户端 ====")
    print("支持的命令:")
    print("  A/B/C/D - 执行对应任务")

    print("  q/quit - 退出")
    
    while not rospy.is_shutdown():
        # 获取用户输入
        command = input("\n请输入命令: ").upper().strip()
        
        # 检查退出条件
        if command in ['Q', 'QUIT']:
            print("客户端已退出")
            break
            
        # 处理强制导航命令
        if command.startswith('F'):
            try:
                parts = command.split()
                if len(parts) != 2:
                    print("❌ 格式错误: F [索引]")
                    continue
                    
                wp_idx = int(parts[1])
                print(f"正在强制导航到索引 {wp_idx}...")
                if send_force_navigation(wp_idx):
                    print("✅ 导航请求已发送")
                else:
                    print("❌ 请求失败，请重试")
                    
            except ValueError:
                print("❌ 索引必须是数字")
            continue
            
        # 处理标准任务命令
        if command in ['A', 'B', 'C', 'D']:
            if send_transport_request(command):
                print("任务已提交，机器人执行完毕")
            else:
                print("❌ 请求失败，请重试")
            continue
            
        # 未知命令
        print("❌ 未知命令，请重试")
