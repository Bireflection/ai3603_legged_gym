# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class GO1RoughCfg( LeggedRobotCfg ):
    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.34] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'FL_hip_joint': 0.1,   # [rad]
            'RL_hip_joint': 0.1,   # [rad]
            'FR_hip_joint': -0.1 ,  # [rad]
            'RR_hip_joint': -0.1,   # [rad]

            'FL_thigh_joint': 0.8,     # [rad]
            'RL_thigh_joint': 1.,   # [rad]
            'FR_thigh_joint': 0.8,     # [rad]
            'RR_thigh_joint': 1.,   # [rad]

            'FL_calf_joint': -1.5,   # [rad]
            'RL_calf_joint': -1.5,    # [rad]
            'FR_calf_joint': -1.5,  # [rad]
            'RR_calf_joint': -1.5,    # [rad]
        }

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        stiffness = {'joint': 20.}  # [N*m/rad]
        damping = {'joint': 0.5}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # hip_scale_reduction = 0.5
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/go1/urdf/go1.urdf'
        name = "go1"
        foot_name = "foot"
        penalize_contacts_on = ["thigh", "calf"]
        terminate_after_contacts_on = ["base"]
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter
        # flip_visual_attachments = False
        # fix_base_link = False
  
    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.9
        base_height_target = 0.25
        tracking_sigma = 0.25  # 减小以提高准确性
        class scales( LeggedRobotCfg.rewards.scales ):
            # tracking_lin_vel = 1.5  # 增加以提高准确性和敏捷性
            # tracking_ang_vel = 0.75  # 增加以提高准确性
            # tracking_lin_vel = 2.0  # 增加以提高准确性和敏捷性 finetune 1234
            tracking_lin_vel = 3.0  # 增加以提高准确性和敏捷性 finetune 56
            tracking_ang_vel = 0.9  # 增加以提高准确性
            tracking_x_vel = 5.0
            # tracking_y_vel = 1.0 base finetune1
            # tracking_y_vel = 2.0 # finetune 2
            tracking_y_vel = 3.0 # finetune 3 4 5
            yaw = 1.0
            # clip_y = 1000.0 # finetune 7clip
            # tracking_x_acc = 1.0 
            stable_acc = 1.0 # finetune4
            lin_vel_z = -4  # 减少以提高稳定性
            ang_vel_xy = -0.025  # 减少以提高稳定性
            orientation = -0.5  # 增加以提高稳定性
            torques = -2.5e-5
            dof_acc = -1.25e-7
            feet_air_time = 1.0
            dof_pos_limits = -10.0
    class commands( LeggedRobotCfg.commands ):
        curriculum = False
        class ranges( LeggedRobotCfg.commands.ranges ):
            # lin_vel_x = [-1.0, 2.0] # min max [m/s] base
            lin_vel_x = [-1.0, 3.0] # min max [m/s] finetune1235 7 normal
            # lin_vel_x = [-1.0, 1.0] # min max [m/s] finetune467clip
            # lin_vel_y = [0.0, 0.0] # min max [m/s] finetune467clip
    # class terrain( LeggedRobotCfg.terrain ):
    #     mesh_type = 'plane' # "heightfield" # none, plane, heightfield or trimeshfinetune2
class GO1RoughCfgPPO( LeggedRobotCfgPPO ):
    class policy( LeggedRobotCfgPPO.policy ):
        activation = 'elu'
        # rnn_type = 'gru'
        # rnn_hidden_size = 512
        # rnn_num_layers = 1
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
        # learning_rate = 1.e-3 base finetune1
        learning_rate = 1.e-4 # finetune2
        lam = 0.97
    class runner( LeggedRobotCfgPPO.runner ):
        # policy_class_name = 'ActorCriticRecurrent'
        run_name = ''
        experiment_name = 'go1'
        # max_iterations = 5000
        max_iterations = 10000

