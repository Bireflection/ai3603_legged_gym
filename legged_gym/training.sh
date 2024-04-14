#!/bin/bash

echo "Start Training..."

for value in $(seq 1 0.1 2)
do
    echo "run train.py with tracking_lin_vel=$value"
    python ./legged_gym/scripts/train.py --task=go1 --num_envs=1024 --headless --tracking_lin_vel=$value
done

for value in $(seq 0.1 0.1 1)
do
    echo "run train.py with tracking_ang_vel=$value"
    python ./legged_gym/scripts/train.py --task=go1 --num_envs=1024 --headless --tracking_ang_vel=$value
done

for value in $(seq 5 1 8)
do
    echo "run train.py with lr=$value"
    python ./legged_gym/scripts/train.py --task=go1 --num_envs=1024 --headless --tracking_x_vel=$value
done

for value in $(seq 0.95 0.01 0.98)
do
    echo "run train.py with lam=$value"
    python ./legged_gym/scripts/train.py --task=go1 --num_envs=1024 --headless --lam=$value
done

for value in $(seq 0.1 0.1 1)
do
    echo "run train.py with tracking_y_vel=$value"
    python ./legged_gym/scripts/train.py --task=go1 --num_envs=1024 --headless --tracking_y_vel=$value
done

for value in $(seq 0.05 0.05 0.3)
do
    echo "run train.py with tracking_sigma=$value"
    python ./legged_gym/scripts/train.py --task=go1 --num_envs=1024 --headless --tracking_sigma=$value
done

echo "Training Finish."
