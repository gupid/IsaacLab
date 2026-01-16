"""Script to test environment registration and visualization."""

# # Joint Position Control 关节位置控制
#     id="Isaac-Lift-Cube-Franka-v0",
#     id="Isaac-Lift-Cube-Franka-Play-v0",
# # Inverse Kinematics - Absolute Pose Control逆运动学 - 绝对位姿控制
#     id="Isaac-Lift-Cube-Franka-IK-Abs-v0",
#     id="Isaac-Lift-Teddy-Bear-Franka-IK-Abs-v0",
# # Inverse Kinematics - Relative Pose Control逆运动学 - 相对位姿控制
#     id="Isaac-Lift-Cube-Franka-IK-Rel-v0",
# # Camera-based Joint Position Control 摄像头关节位置控制
#     id="Isaac-Lift-Cube-Franka-RGB-v0",
#     id="Isaac-Lift-Cube-Franka-Depth-v0",
#     id="Isaac-Lift-Cube-Franka-RGB-ResNet18-v0",
#     id="Isaac-Lift-Cube-Franka-RGB-TheiaTiny-v0",

# # 推荐用法
# python test.py
# python test.py --task Isaac-Lift-Cube-Franka-Play-v0
# python test.py --task Isaac-Lift-Cube-Franka-IK-Abs-v0
# python test.py --task Isaac-Lift-Teddy-Bear-Franka-IK-Abs-v0
# python test.py --task Isaac-Lift-Cube-Franka-IK-Rel-v0

# python test.py --task Isaac-Lift-Cube-Franka-RGB-v0 --enable_cameras
# python test.py --task Isaac-Lift-Cube-Franka-Depth-v0 --enable_cameras
# python test.py --task Isaac-Lift-Cube-Franka-RGB-ResNet18-v0 --enable_cameras
# python test.py --task Isaac-Lift-Cube-Franka-RGB-TheiaTiny-v0 --enable_cameras

import argparse
import sys
import traceback

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Test environment registration and visualization.")
parser.add_argument("--task", type=str, default="Isaac-Lift-Cube-Franka-v0", help="Name of the task.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli, hydra_args = parser.parse_known_args()

# clear out sys.argv for Hydra
sys.argv = [sys.argv[0]] + hydra_args

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import time
import torch
import numpy as np

import isaaclab_tasks  # noqa: F401

# Mapping task names to config modules
TASK_CONFIG_MAP = {
    "Isaac-Lift-Cube-Franka-v0": "isaaclab_tasks.manager_based.manipulation.lift.config.franka.joint_pos_env_cfg:FrankaCubeLiftEnvCfg",
    "Isaac-Lift-Cube-Franka-Play-v0": "isaaclab_tasks.manager_based.manipulation.lift.config.franka.joint_pos_env_cfg:FrankaCubeLiftEnvCfg_PLAY",
    "Isaac-Lift-Cube-Franka-RGB-v0": "isaaclab_tasks.manager_based.manipulation.lift.config.franka.camera_joint_pos_env_cfg:FrankaCubeLiftRGBCameraEnvCfg",
    "Isaac-Lift-Cube-Franka-Depth-v0": "isaaclab_tasks.manager_based.manipulation.lift.config.franka.camera_joint_pos_env_cfg:FrankaCubeLiftDepthCameraEnvCfg",
    "Isaac-Lift-Cube-Franka-RGB-ResNet18-v0": "isaaclab_tasks.manager_based.manipulation.lift.config.franka.camera_joint_pos_env_cfg:FrankaCubeLiftResNet18CameraEnvCfg",
    "Isaac-Lift-Cube-Franka-RGB-TheiaTiny-v0": "isaaclab_tasks.manager_based.manipulation.lift.config.franka.camera_joint_pos_env_cfg:FrankaCubeLiftTheiaTinyCameraEnvCfg",
    "Isaac-Lift-Cube-Franka-IK-Abs-v0": "isaaclab_tasks.manager_based.manipulation.lift.config.franka.ik_abs_env_cfg:FrankaCubeLiftEnvCfg",
    "Isaac-Lift-Teddy-Bear-Franka-IK-Abs-v0": "isaaclab_tasks.manager_based.manipulation.lift.config.franka.ik_abs_env_cfg:FrankaTeddyBearLiftEnvCfg",
    "Isaac-Lift-Cube-Franka-IK-Rel-v0": "isaaclab_tasks.manager_based.manipulation.lift.config.franka.ik_rel_env_cfg:FrankaCubeLiftEnvCfg",
}

# specify rendering parameters
render_mode = None if args_cli.headless else "rgb_array"

try:
    print(f"[INFO] Creating environment: {args_cli.task}")
    print(f"[INFO] Number of environments: {args_cli.num_envs}")
    
    # Get config path from mapping
    if args_cli.task not in TASK_CONFIG_MAP:
        print(f"[ERROR] Task '{args_cli.task}' not found. Available tasks:")
        for task_name in TASK_CONFIG_MAP.keys():
            print(f"  - {task_name}")
        sys.exit(1)
    
    config_path = TASK_CONFIG_MAP[args_cli.task]
    module_path, class_name = config_path.split(":")
    
    # Dynamically import config class
    module = __import__(module_path, fromlist=[class_name])
    env_cfg_class = getattr(module, class_name)
    
    # Create environment config
    env_cfg = env_cfg_class()
    env_cfg.scene.num_envs = args_cli.num_envs
    
    # create environment
    env = gym.make(
        args_cli.task,
        cfg=env_cfg,
        render_mode=render_mode,
    )
    
    print("[INFO] Environment created successfully!")
    print(f"[INFO] Observation space: {env.observation_space}")
    print(f"[INFO] Action space: {env.action_space}")
    
    # Get unwrapped environment to access device
    env_unwrapped = env.unwrapped
    device = env_unwrapped.device
    
    # reset environment
    print("\n[INFO] Resetting environment...")
    obs, info = env.reset()
    print(f"[INFO] Observation type: {type(obs)}")
    if isinstance(obs, dict):
        for key, value in obs.items():
            if isinstance(value, torch.Tensor):
                print(f"[INFO]   - {key}: {value.shape}")
            else:
                print(f"[INFO]   - {key}: {value}")
    else:
        print(f"[INFO] Observation shape: {obs.shape}")
    
    # run simulation without actions
    print("\n[INFO] Running simulation without actions (idle visualization)...")
    step_count = 0
    
    while simulation_app.is_running():
        # get zero actions as tensor
        actions = torch.zeros(env.action_space.shape, dtype=torch.float32, device=device)
        
        # step the environment
        obs, reward, terminated, truncated, info = env.step(actions)
        step_count += 1
        
        # render
        if not args_cli.headless:
            env.render()
    
    print(f"\n[INFO] Simulation completed successfully!")
    print(f"[INFO] Total steps: {step_count}")
    
    env.close()
    print("[INFO] Environment closed.")

except Exception as e:
    print(f"[ERROR] Failed to create or run environment: {e}")
    traceback.print_exc()
    sys.exit(1)

finally:
    # close sim app
    simulation_app.close()