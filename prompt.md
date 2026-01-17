工程结构概览（IsaacLab）

顶层目录
- apps/：IsaacSim/IsaacLab 应用配置（多种 .kit 启动配置、headless/rendering/XR 变体）。
- source/：核心源码与子包集合（isaaclab、isaaclab_tasks、isaaclab_rl、isaaclab_mimic、isaaclab_assets 等）。
- scripts/：常用运行入口与示例脚本（tutorials、demos、reinforcement_learning、imitation_learning、environments 等）。
- tools/：测试与工具脚本（run_all_tests.py、run_train_envs.py、install_deps.py 等）。
- docs/：Sphinx 文档与说明。
- docker/：容器相关配置。
- isaaclab.sh / isaaclab.bat：Linux/Windows 启动脚本。
- 其他元数据：pyproject.toml、environment.yml、pytest.ini、README.md、LICENSE* 等。

核心包结构（source/）
- source/isaaclab/isaaclab/：核心仿真与组件库
  - envs/（环境基类与封装）
  - scene/（场景与实体组织）
  - sim/（仿真接口）
  - sensors/、actuators/、controllers/
  - assets/、terrains/、utils/、ui/ 等
- source/isaaclab_tasks/isaaclab_tasks/：任务与环境集合
  - direct/（直接式任务）
  - manager_based/（管理器式任务）
  - utils/
- source/isaaclab_rl/isaaclab_rl/：RL 框架集成
  - rl_games/、rsl_rl/、sb3.py、skrl.py、utils/
- source/isaaclab_mimic/isaaclab_mimic/：模仿学习/数据生成
  - datagen/、motion_planners/、envs/、ui/
- source/isaaclab_assets/isaaclab_assets/：资源与资产
  - robots/、sensors/

常见开发入口
- 新增任务/环境：优先在 source/isaaclab_tasks/isaaclab_tasks/ 下扩展。
- 训练/运行示例：scripts/ 下对应领域目录。
- 测试与工具：tools/ 与各包的 test/ 目录。

结构：
```
.
├── apps
│   ├── isaaclab.python.headless.kit
│   ├── isaaclab.python.headless.rendering.kit
│   ├── isaaclab.python.kit
│   ├── isaaclab.python.rendering.kit
│   ├── isaaclab.python.xr.openxr.headless.kit
│   ├── isaaclab.python.xr.openxr.kit
│   ├── isaacsim_4_5
│   │   ├── extension.toml
│   │   ├── isaaclab.python.headless.kit
│   │   ├── isaaclab.python.headless.rendering.kit
│   │   ├── isaaclab.python.kit
│   │   ├── isaaclab.python.rendering.kit
│   │   ├── isaaclab.python.xr.openxr.headless.kit
│   │   ├── isaaclab.python.xr.openxr.kit
│   │   └── rendering_modes
│   └── rendering_modes
│       ├── balanced.kit
│       ├── extension.toml
│       ├── performance.kit
│       └── quality.kit
├── CITATION.cff
├── CONTRIBUTING.md
├── CONTRIBUTORS.md
├── docker
│   ├── cluster
│   │   ├── cluster_interface.sh
│   │   ├── run_singularity.sh
│   │   ├── submit_job_pbs.sh
│   │   └── submit_job_slurm.sh
│   ├── container.py
│   ├── container.sh
│   ├── docker-compose.cloudxr-runtime.patch.yaml
│   ├── docker-compose.yaml
│   ├── Dockerfile.base
│   ├── Dockerfile.curobo
│   ├── Dockerfile.ros2
│   ├── test
│   │   └── test_docker.py
│   ├── utils
│   │   ├── container_interface.py
│   │   ├── __init__.py
│   │   ├── state_file.py
│   │   └── x11_utils.py
│   └── x11.yaml
├── docs
│   ├── conf.py
│   ├── index.rst
│   ├── licenses
│   │   ├── assets
│   │   └── dependencies
│   ├── make.bat
│   ├── Makefile
│   ├── README.md
│   ├── _redirect
│   │   └── index.html
│   ├── requirements.txt
│   ├── source
│   │   ├── api
│   │   ├── deployment
│   │   ├── experimental-features
│   │   ├── features
│   │   ├── how-to
│   │   ├── migration
│   │   ├── overview
│   │   ├── policy_deployment
│   │   ├── refs
│   │   ├── setup
│   │   ├── _static
│   │   └── tutorials
│   └── _templates
│       └── versioning.html
├── environment.yml
├── greptile.json
├── isaaclab.bat
├── isaaclab.sh
├── _isaac_sim -> /home/gupid/isaacsim
├── LICENSE
├── LICENSE-mimic
├── prompt.md
├── pyproject.toml
├── pytest.ini
├── README.md
├── scripts
│   ├── benchmarks
│   │   ├── benchmark_cameras.py
│   │   ├── benchmark_load_robot.py
│   │   ├── benchmark_non_rl.py
│   │   ├── benchmark_rlgames.py
│   │   ├── benchmark_rsl_rl.py
│   │   ├── benchmark_view_comparison.py
│   │   ├── benchmark_xform_prim_view.py
│   │   └── utils.py
│   ├── demos
│   │   ├── arms.py
│   │   ├── bin_packing.py
│   │   ├── bipeds.py
│   │   ├── deformables.py
│   │   ├── h1_locomotion.py
│   │   ├── hands.py
│   │   ├── haply_teleoperation.py
│   │   ├── markers.py
│   │   ├── multi_asset.py
│   │   ├── pick_and_place.py
│   │   ├── procedural_terrain.py
│   │   ├── quadcopter.py
│   │   ├── quadrupeds.py
│   │   └── sensors
│   ├── environments
│   │   ├── export_IODescriptors.py
│   │   ├── list_envs.py
│   │   ├── random_agent.py
│   │   ├── state_machine
│   │   ├── teleoperation
│   │   └── zero_agent.py
│   ├── imitation_learning
│   │   ├── isaaclab_mimic
│   │   ├── locomanipulation_sdg
│   │   └── robomimic
│   ├── reinforcement_learning
│   │   ├── ray
│   │   ├── rl_games
│   │   ├── rsl_rl
│   │   ├── sb3
│   │   └── skrl
│   ├── sim2sim_transfer
│   │   ├── config
│   │   └── rsl_rl_transfer.py
│   ├── tools
│   │   ├── blender_obj.py
│   │   ├── check_instanceable.py
│   │   ├── convert_instanceable.py
│   │   ├── convert_mesh.py
│   │   ├── convert_mjcf.py
│   │   ├── convert_urdf.py
│   │   ├── cosmos
│   │   ├── hdf5_to_mp4.py
│   │   ├── merge_hdf5_datasets.py
│   │   ├── mp4_to_hdf5.py
│   │   ├── process_meshes_to_obj.py
│   │   ├── record_demos.py
│   │   ├── replay_demos.py
│   │   ├── test
│   │   └── train_and_publish_checkpoints.py
│   └── tutorials
│       ├── 00_sim
│       ├── 01_assets
│       ├── 02_scene
│       ├── 03_envs
│       ├── 04_sensors
│       └── 05_controllers
├── SECURITY.md
├── source
│   ├── isaaclab
│   │   ├── config
│   │   ├── docs
│   │   ├── isaaclab
│   │   ├── isaaclab.egg-info
│   │   ├── pyproject.toml
│   │   ├── setup.py
│   │   └── test
│   ├── isaaclab_assets
│   │   ├── config
│   │   ├── data
│   │   ├── docs
│   │   ├── isaaclab_assets
│   │   ├── isaaclab_assets.egg-info
│   │   ├── pyproject.toml
│   │   ├── setup.py
│   │   └── test
│   ├── isaaclab_mimic
│   │   ├── config
│   │   ├── docs
│   │   ├── isaaclab_mimic
│   │   ├── isaaclab_mimic.egg-info
│   │   ├── pyproject.toml
│   │   ├── setup.py
│   │   └── test
│   ├── isaaclab_rl
│   │   ├── config
│   │   ├── docs
│   │   ├── isaaclab_rl
│   │   ├── isaaclab_rl.egg-info
│   │   ├── pyproject.toml
│   │   ├── setup.py
│   │   └── test
│   └── isaaclab_tasks
│       ├── config
│       ├── docs
│       ├── isaaclab_tasks
│       ├── isaaclab_tasks.egg-info
│       ├── pyproject.toml
│       ├── setup.py
│       └── test
├── tools
│   ├── conftest.py
│   ├── install_deps.py
│   ├── run_all_tests.py
│   ├── run_train_envs.py
│   ├── template
│   │   ├── cli.py
│   │   ├── common.py
│   │   ├── generator.py
│   │   ├── __init__.py
│   │   ├── requirements.txt
│   │   └── templates
│   └── test_settings.py
└── VERSION
```

```shell
IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/lift
.
├── config
│   ├── franka
│   │   ├── agents
│   │   │   ├── __init__.py
│   │   │   ├── __pycache__
│   │   │   │   ├── __init__.cpython-310.pyc
│   │   │   │   └── rsl_rl_ppo_cfg.cpython-310.pyc
│   │   │   ├── rl_games_ppo_cfg.yaml
│   │   │   ├── robomimic
│   │   │   │   ├── bc.json
│   │   │   │   └── bcq.json
│   │   │   ├── rsl_rl_ppo_cfg.py
│   │   │   ├── sb3_ppo_cfg.yaml
│   │   │   └── skrl_ppo_cfg.yaml
│   │   ├── ik_abs_env_cfg.py
│   │   ├── ik_rel_env_cfg.py
│   │   ├── __init__.py
│   │   ├── joint_pos_env_cfg.py
│   │   └── __pycache__
│   │       ├── __init__.cpython-310.pyc
│   │       └── joint_pos_env_cfg.cpython-310.pyc
│   ├── __init__.py
│   ├── openarm
│   │   ├── agents
│   │   │   ├── __init__.py
│   │   │   ├── __pycache__
│   │   │   │   └── __init__.cpython-310.pyc
│   │   │   ├── rl_games_ppo_cfg.yaml
│   │   │   └── rsl_rl_ppo_cfg.py
│   │   ├── __init__.py
│   │   ├── joint_pos_env_cfg.py
│   │   ├── lift_openarm_env_cfg.py
│   │   └── __pycache__
│   │       └── __init__.cpython-310.pyc
│   └── __pycache__
│       └── __init__.cpython-310.pyc
├── __init__.py
├── lift_env_cfg.py
├── mdp
│   ├── __init__.py
│   ├── observations.py
│   ├── __pycache__
│   │   ├── __init__.cpython-310.pyc
│   │   ├── observations.cpython-310.pyc
│   │   ├── rewards.cpython-310.pyc
│   │   └── terminations.cpython-310.pyc
│   ├── rewards.py
│   └── terminations.py
└── __pycache__
    ├── __init__.cpython-310.pyc
    └── lift_env_cfg.cpython-310.pyc
```

