# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Common functions that can be used to create curriculum for the learning environment.

The functions can be passed to the :class:`isaaclab.managers.CurriculumTermCfg` object to enable
the curriculum introduced by the function.
"""

from __future__ import annotations

import re
from collections.abc import Sequence
from typing import TYPE_CHECKING, ClassVar

import torch

from isaaclab.managers import CurriculumTermCfg, ManagerTermBase

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

class modify_reward_weight_multi(ManagerTermBase):
    """Curriculum that modifies the reward weight based on multiple step-wise stages."""

    def __init__(self, cfg: CurriculumTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)

        # obtain term configuration
        term_name = cfg.params["term_name"]
        self._term_cfg = env.reward_manager.get_term_cfg(term_name)
        
        # 解析阶段参数
        self.schedule = self._parse_schedule(cfg.params)

    def _parse_schedule(self, params):
        """解析调度参数"""
        schedule = []
        
        # 处理单阶段模式（向后兼容）
        if "weight" in params and "num_steps" in params:
            return [(params["num_steps"], params["weight"])]
        
        # 处理多阶段模式
        if "schedule" in params:
            for stage in params["schedule"]:
                if isinstance(stage, dict) and "num_steps" in stage and "weight" in stage:
                    schedule.append((stage["num_steps"], stage["weight"]))
                elif isinstance(stage, (list, tuple)) and len(stage) == 2:
                    schedule.append((stage[0], stage[1]))
        
        # 按步数阈值排序
        schedule.sort(key=lambda x: x[0])
        return schedule

    def __call__(
        self,
        env: ManagerBasedRLEnv,
        env_ids: Sequence[int],
        term_name: str,
        schedule: list,  # [(num_steps, weight), ...] 或 [{"num_steps": x, "weight": y}, ...]
    ) -> float:
        # 如果没有阶段定义，使用默认参数（向后兼容）
        if not self.schedule:
            if "weight" in self._cfg.params and "num_steps" in self._cfg.params:
                weight = self._cfg.params["weight"]
                num_steps = self._cfg.params["num_steps"]
                if env.common_step_counter > num_steps:
                    self._term_cfg.weight = weight
                    env.reward_manager.set_term_cfg(term_name, self._term_cfg)
            return self._term_cfg.weight
        
        # 多阶段处理
        current_steps = env.common_step_counter
        current_weight = self._term_cfg.weight
        
        # 查找当前应该使用的权重
        for step_threshold, weight in self.schedule:
            if current_steps >= step_threshold:
                current_weight = weight
            else:
                break
        
        # 如果权重有变化，更新配置
        if current_weight != self._term_cfg.weight:
            self._term_cfg.weight = current_weight
            env.reward_manager.set_term_cfg(term_name, self._term_cfg)
        
        return current_weight