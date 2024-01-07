# AI3603: Legged gym

This is the code base of Robot Control with Reinforcement Learning based on Isaac Gym Environments for Unitree Go1 Robots. Our focus is on training the Unitree Go1 quadruped robot to proficiently follow given speed commands, aiming to improve its accuracy, agility, and stability.

This is the final project for AI3603 course of Shanghai Jiao Tong University.

The lecturer of this course is Professor Yue Gao.

## Structure
The final model is stored in `./legged_gym/legged_gym/logs/go1/Jan05_16-03-14_/35000.pt`. Other attempts are stored in `Previous Training` folder. Videos of training and evaluating is stored in `Video` folder.

## Usage
You can simply run the `./legged_gym/legged_gym/test_acc.sh` or other scripts to test the outcome. 

Or you can use the following command to run:

```
python ./legged_gym/legged_gym/scripts/play.py --task=go1 --num_envs=num --type
```

where `num` represents the number of environments, and `type` represents the task to evaluate, with `accuracy`, `agility` and `stability` to choose.
