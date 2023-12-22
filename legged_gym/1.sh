#!/bin/bash

# 脚本开始
echo "开始运行脚本..."

# # 循环从 1 到 2，步长为 0.1
# for value in $(seq 1 0.1 2)
# do
#     # 执行 Python 脚本，带有不同的 tracking_lin_vel 参数
#     echo "运行 train.py with tracking_lin_vel=$value"
#     python ./legged_gym/scripts/train.py --task=go1 --num_envs=1024 --headless --tracking_lin_vel=$value
# done

# for value in $(seq 0.1 0.1 1)
# do
#     # 执行 Python 脚本，带有不同的 tracking_lin_vel 参数
#     echo "运行 train.py with tracking_ang_vel=$value"
#     python ./legged_gym/scripts/train.py --task=go1 --num_envs=1024 --headless --tracking_ang_vel=$value
# done

for value in $(seq 0.1 0.1 1)
do
    # 执行 Python 脚本，带有不同的 tracking_lin_vel 参数
    echo "运行 train.py with tracking_x_vel=$value"
    python ./legged_gym/scripts/train.py --task=go1 --num_envs=1024 --headless --tracking_x_vel=$value
done
for value in $(seq 0.1 0.1 1)
do
    # 执行 Python 脚本，带有不同的 tracking_lin_vel 参数
    echo "运行 train.py with tracking_y_vel=$value"
    python ./legged_gym/scripts/train.py --task=go1 --num_envs=1024 --headless --tracking_y_vel=$value
done
# for value in $(seq 0.05 0.05 0.3)
# do
#     # 执行 Python 脚本，带有不同的 tracking_lin_vel 参数
#     echo "运行 train.py with tracking_sigma=$value"
#     python ./legged_gym/scripts/train.py --task=go1 --num_envs=1024 --headless --tracking_sigma=$value
# done

echo "脚本运行完成."
