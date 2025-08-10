#!/bin/bash

#用途：一键启动仿真任务

execute_command() {
    command="$1"
    title="$2"
    echo "正在执行: $title"
    eval "$command"
    if [ $? -ne 0 ]; then
        echo "错误: $title 执行失败。"
        exit 1
    fi
}

# 打开新终端并执行 roslaunch 命令
execute_command "gnome-terminal -- bash -c 'source devel/setup.bash; roslaunch gazebo_nav gazebo_nav.launch; exec bash'" "启动 导航"

sleep 5

execute_command "gnome-terminal --tab -- bash -c 'cd ..; cd yolov5-master/; source ~/miniconda3/etc/profile.d/conda.sh && conda activate yolov5 && python3 auto_detect.py; exec bash'" "运行 yolo识别 脚本"

sleep 3

execute_command "gnome-terminal --tab -- bash -c 'cd ..; python3 test_ws/src/scripts/start.py; exec bash'" "启动 发点程序"


echo "所有命令执行完毕。"
