#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header, ColorRGBA

def create_letter_marker(text, position, marker_id=0, size=0.2, color=(1.0, 0.0, 0.0, 1.0), frame_id="map"):
    """
    创建一个文本标记消息（支持单个字母或单词）
    
    参数:
    text (str): 要显示的文本（可以是单个字母或单词）
    position (tuple): 文本显示位置的(x, y, z)坐标
    marker_id (int): 标记的唯一ID
    size (float): 文本大小
    color (tuple): 文本颜色，格式为(r, g, b, a)
    frame_id (str): 参考坐标系
    
    返回:
    Marker: 配置好的文本标记消息
    """
    # 验证输入是否为字符串
    if not isinstance(text, str):
        rospy.logwarn(f"输入类型错误，应为字符串，将默认显示 'A'")
        text = 'A'
    
    marker = Marker(
        header=Header(frame_id=frame_id, stamp=rospy.Time.now()),
        ns="text_markers",
        id=marker_id,
        type=Marker.TEXT_VIEW_FACING,
        action=Marker.ADD,
        pose=Pose(
            position=Point(x=position[0], y=position[1], z=position[2]),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        ),
        scale=Point(x=size, y=size, z=size),  # 统一缩放三个维度
        color=ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3]),
        text=text  # 直接使用输入的文本，不再强制转为大写
    )
    return marker

def publish_text_markers(text_infos, rate=1.0):
    """
    发布多个文本标记到rviz
    
    参数:
    text_infos (list): 包含文本信息的列表，每个元素为(text, position, size, color)
    rate (float): 发布频率(Hz)
    """
    # 初始化ROS节点
    rospy.init_node('text_marker_publisher', anonymous=True)
    
    # 创建Marker发布者
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    
    # 设置发布频率
    rate = rospy.Rate(rate)
    
    # 持续发布标记，直到节点关闭
    while not rospy.is_shutdown():
        # 为每个文本信息创建并发布标记
        for i, (text, position, size, color) in enumerate(text_infos):
            marker = create_letter_marker(text, position, marker_id=i, size=size, color=color)
            marker.header.stamp = rospy.Time.now()
            marker_pub.publish(marker)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        # 设置多个要显示的文本信息，格式为：(文本, (x, y, z坐标), 字号, (r, g, b, a颜色))
        text_infos = [
            ('A', (1.52, -0.39, 0.5), 0.7, (1.0, 0.0, 0.0, 1.0)),  # 红色A
            ('B', (-7.63, -1.0, 0.5), 0.7, (0.0, 1.0, 0.0, 1.0)),  # 绿色B
            ('C', (-8.32, 4.31, 0.5), 0.7, (0.0, 0.0, 1.0, 1.0)),  # 蓝色C
            ('D', (-4.9, 3.86, 0.5), 0.7, (1.0, 1.0, 0.0, 1.0)), # 黄色D
            ('Wait', (-0.15, 1.1, 0.5), 0.5, (0.0, 1.0, 1.0, 1.0)),  # 青色
            ('Camp', (1.22, 1.1, 0.5), 0.5, (1.0, 1.0, 1.0, 1.0)),  #白色
        ]
        
        # 发布多个文本标记
        publish_text_markers(text_infos)
    except rospy.ROSInterruptException:
        pass
