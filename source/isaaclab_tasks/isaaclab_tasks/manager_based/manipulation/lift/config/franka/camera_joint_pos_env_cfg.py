# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Camera-based environment configuration for Franka cube lift task."""

from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ObservationGroupCfg, ObservationTermCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import TiledCameraCfg
from isaaclab.utils import configclass
import isaaclab.sim as sim_utils
import isaaclab.envs.mdp as mdp

from .joint_pos_env_cfg import FrankaCubeLiftEnvCfg, FrankaCubeLiftEnvCfg_PLAY


##
# MDP settings
##


@configclass
class RGBObservationsCfg:
    """Observation specifications for the MDP with RGB camera."""

    @configclass
    class RGBCameraPolicyCfg(ObservationGroupCfg):
        """Observations for policy group with RGB images."""

        image = ObservationTermCfg(func=mdp.image, params={"sensor_cfg": SceneEntityCfg("tiled_camera"), "data_type": "rgb"})

    policy: ObservationGroupCfg = RGBCameraPolicyCfg()


@configclass
class DepthObservationsCfg:
    """Observation specifications for the MDP with depth camera."""

    @configclass
    class DepthCameraPolicyCfg(ObservationGroupCfg):
        """Observations for policy group with depth images."""

        image = ObservationTermCfg(
            func=mdp.image, params={"sensor_cfg": SceneEntityCfg("tiled_camera"), "data_type": "distance_to_camera"}
        )

    policy: ObservationGroupCfg = DepthCameraPolicyCfg()


@configclass
class ResNet18ObservationCfg:
    """Observation specifications for the MDP with ResNet18 features."""

    @configclass
    class ResNet18FeaturesCameraPolicyCfg(ObservationGroupCfg):
        """Observations for policy group with features extracted from RGB images with a frozen ResNet18."""

        image = ObservationTermCfg(
            func=mdp.image_features,
            params={"sensor_cfg": SceneEntityCfg("tiled_camera"), "data_type": "rgb", "model_name": "resnet18"},
        )

    policy: ObservationGroupCfg = ResNet18FeaturesCameraPolicyCfg()


@configclass
class TheiaTinyObservationCfg:
    """Observation specifications for the MDP with TheiaTiny features."""

    @configclass
    class TheiaTinyFeaturesCameraPolicyCfg(ObservationGroupCfg):
        """Observations for policy group with features extracted from RGB images with a frozen TheiaTiny."""

        image = ObservationTermCfg(
            func=mdp.image_features,
            params={"sensor_cfg": SceneEntityCfg("tiled_camera"), "data_type": "rgb", "model_name": "theia-tiny-patch16-224-cddsv"},
        )

    policy: ObservationGroupCfg = TheiaTinyFeaturesCameraPolicyCfg()


##
# Environment configuration
##


@configclass
class FrankaCubeLiftRGBCameraEnvCfg(FrankaCubeLiftEnvCfg):
    """Configuration for the Franka cube lift environment with RGB camera."""

    def __post_init__(self):
        super().__post_init__()
        # add camera to the scene
        self.scene.tiled_camera = TiledCameraCfg(
            prim_path="{ENV_REGEX_NS}/Camera",
            offset=TiledCameraCfg.OffsetCfg(pos=(0, 2, 0.4), rot=(0.0, 0.0, 0.70711, -0.70711), convention="ros"),
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=24.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 20.0)
            ),
            width=100,
            height=100,
        )
        # remove ground as it obstructs the camera
        self.scene.ground = None
        # viewer settings
        self.viewer.eye = (0.5, 0.5, 0.5)
        self.viewer.lookat = (0.0, 0.0, 0.2)
        # observations
        self.observations = RGBObservationsCfg()


@configclass
class FrankaCubeLiftDepthCameraEnvCfg(FrankaCubeLiftEnvCfg):
    """Configuration for the Franka cube lift environment with depth camera."""

    def __post_init__(self):
        super().__post_init__()
        # add camera to the scene
        self.scene.tiled_camera = TiledCameraCfg(
            prim_path="{ENV_REGEX_NS}/Camera",
            offset=TiledCameraCfg.OffsetCfg(pos=(0, 2, 0.4), rot=(0.0, 0.0, 0.70711, -0.70711), convention="ros"),
            data_types=["distance_to_camera"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=24.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 20.0)
            ),
            width=100,
            height=100,
        )
        # remove ground as it obstructs the camera
        self.scene.ground = None
        # viewer settings
        self.viewer.eye = (0.5, 0.5, 0.5)
        self.viewer.lookat = (0.0, 0.0, 0.2)
        # observations
        self.observations = DepthObservationsCfg()


@configclass
class FrankaCubeLiftResNet18CameraEnvCfg(FrankaCubeLiftRGBCameraEnvCfg):
    """Configuration for the Franka cube lift environment with ResNet18 features as observations."""

    def __post_init__(self):
        super().__post_init__()
        # observations
        self.observations = ResNet18ObservationCfg()


@configclass
class FrankaCubeLiftTheiaTinyCameraEnvCfg(FrankaCubeLiftRGBCameraEnvCfg):
    """Configuration for the Franka cube lift environment with TheiaTiny features as observations."""

    def __post_init__(self):
        super().__post_init__()
        # observations
        self.observations = TheiaTinyObservationCfg()