#!/bin/bash
# CAN初始化+电机在线检测
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 loopback off
sudo ip link set can0 up
echo "--- can0 已启动，检测电机 ---"
timeout 2 candump can0 &
sleep 0.3
cansend can0 003#FFFFFFFFFFFFFFFC
cansend can0 004#FFFFFFFFFFFFFFFC
sleep 1.7
echo "--- 上面应看到 003 和 004 各回一帧"
# cansend can0 003#FFFFFFFFFFFFFFFD
# cansend can0 004#FFFFFFFFFFFFFFFD

# chmod +x ~/exo_deploy/can_init.sh
# ./can_init.sh    # 以后每次上电跑这一条
