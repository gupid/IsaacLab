# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
This script demonstrates policy inference in a prebuilt USD environment.

In this example, we use a locomotion policy to control the Ant robot. The robot was trained
using skrl. The robot is commanded to move forward.

.. code-block:: bash

    # Run the script
    ./isaaclab.sh -p scripts/tutorials/03_envs/skrl_policy_inference_in_usd.py --checkpoint /path/to/skrl_checkpoint.pt

"""

"""Launch Isaac Sim Simulator first."""


import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on inferencing a skrl-trained policy on an Ant robot in a warehouse.")
parser.add_argument("--checkpoint", type=str, help="Path to skrl model checkpoint (.pt file)", required=True)
parser.add_argument("--format", type=str, choices=["jit", "skrl"], default="auto", 
                    help="Model format: 'jit' for JIT model, 'skrl' for skrl checkpoint")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""
import io
import os
import torch
import torch.nn as nn

import omni

from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from isaaclab_tasks.manager_based.classic.ant.ant_env_cfg import AntEnvCfg


def load_skrl_policy(checkpoint_path, device="cuda:0"):
    """Load skrl policy from checkpoint"""
    print(f"Loading skrl checkpoint: {checkpoint_path}")
    
    # Load checkpoint
    checkpoint = torch.load(checkpoint_path, map_location=device)
    
    if isinstance(checkpoint, dict):
        print(f"Checkpoint keys: {list(checkpoint.keys())}")
        
        # Extract policy weights
        policy_state_dict = None
        if "policy" in checkpoint:
            print("Loading from 'policy' key")
            policy_state_dict = checkpoint["policy"]
        elif "models" in checkpoint and isinstance(checkpoint["models"], dict):
            if "policy" in checkpoint["models"]:
                print("Loading from 'models.policy'")
                policy_state_dict = checkpoint["models"]["policy"]
            elif "actor" in checkpoint["models"]:
                print("Loading from 'models.actor'")
                policy_state_dict = checkpoint["models"]["actor"]
        
        if policy_state_dict is None:
            print("❌ Could not find policy weights")
            return None
        
        # Define network structure (based on skrl output)
        class SkrlAntPolicy(nn.Module):
            def __init__(self, input_dim=60, output_dim=8):
                super().__init__()
                self.shared_net = nn.Sequential(
                    nn.Linear(input_dim, 256),
                    nn.ELU(),
                    nn.Linear(256, 128),
                    nn.ELU(),
                    nn.Linear(128, 64),
                    nn.ELU(),
                )
                self.policy_head = nn.Linear(64, output_dim)
                self.tanh = nn.Tanh()
            
            def forward(self, x):
                features = self.shared_net(x)
                return self.tanh(self.policy_head(features))
        
        # Create model
        model = SkrlAntPolicy().to(device)
        
        # Map weights from skrl format to our model
        weight_mapping = {
            'net_container.0.weight': 'shared_net.0.weight',
            'net_container.0.bias': 'shared_net.0.bias',
            'net_container.2.weight': 'shared_net.2.weight',
            'net_container.2.bias': 'shared_net.2.bias',
            'net_container.4.weight': 'shared_net.4.weight',
            'net_container.4.bias': 'shared_net.4.bias',
            'policy_layer.weight': 'policy_head.weight',
            'policy_layer.bias': 'policy_head.bias',
        }
        
        # Create new state dict
        new_state_dict = {}
        for old_key, new_key in weight_mapping.items():
            if old_key in policy_state_dict:
                new_state_dict[new_key] = policy_state_dict[old_key]
        
        # Load weights
        model.load_state_dict(new_state_dict, strict=False)
        model.eval()
        
        print(f"✅ Skrl policy loaded successfully")
        return model
    
    return None


def main():
    """Main function."""
    # Load the trained policy
    policy_path = os.path.abspath(args_cli.checkpoint)
    
    # Auto-detect format if not specified
    if args_cli.format == "auto":
        # Try to load as JIT first
        try:
            file_content = omni.client.read_file(policy_path)[2]
            file = io.BytesIO(memoryview(file_content).tobytes())
            policy = torch.jit.load(file, map_location=args_cli.device)
            print("✅ Loaded as JIT model")
        except Exception as e:
            print(f"Not a JIT file: {e}")
            print("Trying to load as skrl checkpoint...")
            args_cli.format = "skrl"
    
    # Load based on format
    if args_cli.format == "skrl":
        policy = load_skrl_policy(policy_path, args_cli.device)
        if policy is None:
            print("❌ Failed to load skrl policy, using random actions")
            def random_policy(obs):
                return torch.randn(obs.shape[0], 8).to(args_cli.device)  # Ant has 8 joints
            policy = random_policy
    else:  # JIT format
        file_content = omni.client.read_file(policy_path)[2]
        file = io.BytesIO(memoryview(file_content).tobytes())
        policy = torch.jit.load(file, map_location=args_cli.device)
        print("✅ Loaded as JIT model")

    # Setup environment
    env_cfg = AntEnvCfg()
    env_cfg.scene.num_envs = 1
    env_cfg.curriculum = None
    env_cfg.scene.terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="usd",
        usd_path=f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/warehouse.usd",
    )
    env_cfg.sim.device = args_cli.device
    if args_cli.device == "cpu":
        env_cfg.sim.use_fabric = False

    # Create environment
    env = ManagerBasedRLEnv(cfg=env_cfg)
    print(f"✅ Environment created successfully")

    # Run inference with the policy
    obs, _ = env.reset()
    print(f"Starting inference... Press Ctrl+C to stop")
    
    try:
        with torch.inference_mode():
            step = 0
            while simulation_app.is_running():
                # Get action from policy
                if callable(policy):
                    action = policy(obs["policy"])
                else:
                    action = policy(obs["policy"])
                
                # Step environment
                obs, reward, terminated, truncated, info = env.step(action)
                
                # Print info every 20 steps
                if step % 20 == 0:
                    print(f"Step {step}: reward = {reward.item():.3f}")
                
                # Reset if episode ends
                if terminated or truncated:
                    print("Episode ended, resetting...")
                    obs, _ = env.reset()
                
                step += 1
                
    except KeyboardInterrupt:
        print("\nInference stopped by user")
    except Exception as e:
        print(f"Error during inference: {e}")


if __name__ == "__main__":
    main()
    simulation_app.close()