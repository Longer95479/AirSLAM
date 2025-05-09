整体框架，虚线框为增加的部分：

<p align="middle">
  <img src="figures/system_overview.png" width="600" />
</p>

实现效果如下图：

<p align="middle">
  <img src="figures/MH04_visualization.png" width="600" />
</p>

- 针对传统方法提取的线特征较杂乱且三角化结果较差的问题，基于共识投票策略，在 AirSLAM 上开发了提取场景结构主方向的前端模块，实现场景结构线主方向的估计。
该模块实现了多个水平主方向的检测，且可扩展到对倾斜主方向
的提取，不依赖于特定的线特征，示意如下：

<p align="middle">
  <img src="figures/oringin_lines.jpg" width="400" />
  <img src="figures/DDs.jpg" width="400" />
</p>

- 某帧内的局部主方向与该帧的姿态估计受噪声影响，由此计算出的全局主方向将会偏离真值，为估计出准确的全局主方向，使用卡尔曼滤波实现对全局主方向的微调。相比单次观测，该功能可统计主方向的协方差矩阵，用于外点的剔除。我们能够得到调整后的主方向，以及对应的协方差矩阵直观来看，统计次数越多，协方差越小，越可信

- 主方向约束参与的后端优化。简单来说，将空间中的线段在优化的过程中对齐到主方向。
  - 蓝色节点为位姿；粉色节点为线的3D表示；黑色节点为约束：IMU 预积分约束，线重投影约束，主方向约束

<p align="middle">
  <img src="figures/factor_graph.png" width="250" />
</p>

实验结果：

- ori 表示使用点特征和线特征，pts 表示仅使用点特征，regu 表示使用点特征、线特征、后端增加主方向约束。加粗表示最佳结果

  - 利用线特征提取主方向，并用主方向筛选参与优化的线段，并在优化时加入主方向的约束，能够提高定位精度

<p align="middle">
  <img src="figures/rmse.png" width="400" />
</p>

  - （不提取主方向） VS （提取主方向 + 筛除不属于任意主方向的线段 + 后端增加主方向约束）

<p align="middle">
  <img src="figures/noDDs_vs_DDs_constrain_optim-0.png" width="850" />
</p>

<p align="middle">
  <img src="figures/noDDs_vs_DDs_constrain_optim-1.png" width="850" />
</p>

 - （提取主方向 + 筛除不属于任意主方向的线段 + 后端增加主方向约束） VS （提取主方向 + 筛除不属于任意主方向的线段）

<p align="middle">
  <img src="figures/DDs_constrain_optim_vs_only_DDs_filter.png" width="850" />
</p>


增加的其他功能：

- AirSLAM 在没有时间同步的个人数据集上测试时漂移较大，为了解决IMU与相机时间存在偏移引起
的问题，开发了 IMU 与相机时间戳偏移在线估计功能（[博客：时间戳偏移在线估计](https://longer95479.github.io/temporal-online-calibration-exp)）。
下图展示了 100 ms 偏移情况下，时间偏移估计值是如何收敛到真实偏移附近的：

<p align="middle">
  <img src="figures/temporal_calibration.png" width="500" />
</p>

- AirSLAM 在没有准确外参的个人数据集上测试时漂移较大，为了解决 IMU 与相机外参不准确引起的
问题，开发了 IMU 与相机外参在线估计功能（[博客：外参在线估计](https://longer95479.github.io/extrinsic-calibration-online)）。
使用外参在线估计与不使用外参估计的对比如下：

<p align="middle">
  <img src="figures/not_use_extrinsic_estimate.png" height="250" />
  <img src="figures/use_extrinsic_estimate.png" height="250" />
</p>

- AirSLAM 仅支持 ASL 数据集格式，不方便在 rosbag 数据集的测试，在分析代码中 IMU 预积分与相
机帧的关系后，确定了 IMU 打包策略，在 AirSLAM 上开发了 rosbag 数据集格式的功能。[博客：AirSLAM 代码分析](https://longer95479.github.io/airslam-code-reading)







---


<h1 align="center">AirSLAM: An Efficient and Illumination-Robust Point-Line Visual SLAM System</h1>

<p align="center"><strong>
    <a href = "https://scholar.google.com/citations?user=-p7HvCMAAAAJ&hl=zh-CN">Kuan Xu</a><sup>1</sup>,
    <a href = "https://github.com/yuefanhao">Yuefan Hao</a><sup>2</sup>,
    <a href = "https://scholar.google.com/citations?user=XcV_sesAAAAJ&hl=en">Shenghai Yuan</a><sup>1</sup>,
    <a href = "https://sairlab.org/team/chenw/">Chen Wang</a><sup>2</sup>,
    <a href = "https://scholar.google.com.sg/citations?user=Fmrv3J8AAAAJ&hl=en">Lihua Xie</a><sup>1</sup>
</strong></p>

<p align="center"><strong>
    <a href = "https://www.ntu.edu.sg/cartin">1: Centre for Advanced Robotics Technology Innovation (CARTIN), Nanyang Technological University</a><br>
    <a href = "https://sairlab.org/">2: Spatial AI & Robotics (SAIR) Lab, Computer Science and Engineering, University at Buffalo</a><br>
</strong></p>

<p align="center"><strong> 
    <a href = "https://arxiv.org/pdf/2408.03520">&#128196; [Arxiv]</a> | 
    <a href = "https://xukuanhit.github.io/airslam/">&#128190; [Project Site]</a> |
    <a href = "https://youtu.be/5OcR5KeO5nc">&#127909; [Youtube]</a> |
    <a href = "https://www.bilibili.com/video/BV1rJY7efE9x">&#127909; [Bilibili]</a>
    <!-- &#128214; [OpenAccess] -->
</strong></p>

### :scroll: AirSLAM has dual modes (V-SLAM & VI-SLAM), upgraded from [AirVO (IROS23)](https://github.com/sair-lab/AirSLAM/releases/tag/1.0)

<p align="middle">
  <img src="figures/system_arch.jpg" width="600" />
</p>

**AirSLAM** is an efficient visual SLAM system designed to tackle both short-term and long-term illumination
challenges. Our system adopts a hybrid approach that combines deep learning techniques for feature detection and matching with traditional backend optimization methods. Specifically, we propose a unified convolutional neural network (CNN) that simultaneously extracts keypoints and structural lines. These features are then associated, matched, triangulated, and optimized in a coupled manner. Additionally, we introduce a lightweight relocalization pipeline that reuses the built map, where keypoints, lines, and a structure graph are used to match the query frame with the map. To enhance the applicability of the proposed system to real-world robots, we deploy and accelerate the feature detection and matching networks using C++ and NVIDIA TensorRT. Extensive experiments conducted on various datasets demonstrate that our system outperforms other state-of-the-art visual SLAM systems in illumination-challenging environments. Efficiency evaluations show that our system can run at a rate of 73Hz on a PC and 40Hz on an embedded platform.

**Video**
<p align="middle">
<a href="https://youtu.be/5OcR5KeO5nc" target="_blank"><img src="figures/title.JPG" width="600" border="10"/></a>
</p>


## :eyes: Updates
* [2024.08] We release the code and paper for AirSLAM.
* [2023.07] AriVO is accepted by IROS 2023.
* [2022.10] We release the code and paper for AirVO. The code for AirVO can now be found [here](https://github.com/sair-lab/AirSLAM/tree/airvo_iros).


## :checkered_flag: Test Environment
### Dependencies
* OpenCV 4.2
* Eigen 3
* Ceres 2.0.0
* G2O (tag:20230223_git)
* TensorRT 8.6.1.6
* CUDA 12.1
* python
* ROS noetic
* Boost

### Docker (Recommend)
```bash
docker pull xukuanhit/air_slam:v4
docker run -it --env DISPLAY=$DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix --privileged --runtime nvidia --gpus all --volume ${PWD}:/workspace --workdir /workspace --name air_slam xukuanhit/air_slam:v4 /bin/bash
```

## :book: Data
The data for mapping should be organized in the following Autonomous Systems Lab (ASL) dataset format (imu data is optional):

```
dataroot
├── cam0
│   └── data
│       ├── t0.jpg
│       ├── t1.jpg
│       ├── t2.jpg
│       └── ......
├── cam1
│   └── data
│       ├── t0.jpg
│       ├── t1.jpg
│       ├── t2.jpg
│       └── ......
└── imu0
    └── data.csv

```
After the map is built, the relocalization requires only monocular images. Therefore, you only need to place the query images in a folder.


## :computer: Build
```
    cd ~/catkin_ws/src
    git clone https://github.com/sair-lab/AirSLAM.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## :running: Run 

The launch files for VO/VIO, map optimization, and relocalization are placed in [VO folder](launch/visual_odometry), [MR folder](launch/map_refinement), and [Reloc folder](launch/relocalization), respectively. Before running them, you need to modify the corresponding configurations according to your data path and the desired map-saving path. The following is an example of mapping, optimization, and relocalization with the EuRoC dataset.  


### Mapping
**1**: Change "dataroot" in [VO launch file](launch/visual_odometry/vo_euroc.launch) to your own data path. For the EuRoC dataset, "mav0" needs to be included in the path.

**2**: Change "saving_dir" in the same file to the path where you want to save the map and trajectory. **It must be an existing folder.**

**3**: Run the launch file:

```
roslaunch air_slam vo_euroc.launch 
```

### Map Optimization
**1**: Change "map_root" in [MR launch file](launch/map_refinement/mr_euroc.launch) to your own map path.

**2**: Run the launch file:

```
roslaunch air_slam mr_euroc.launch 
```

### Relocalization
**1**: Change "dataroot" in [Reloc launch file](launch/relocalization/reloc_euroc.launch) to your own query data path.

**2**: Change "map_root" in the same file to your own map path.

**3**: Run the launch file:

```
roslaunch air_slam reloc_euroc.launch 
```

### Other datasets
[Launch folder](launch) and [config folder](configs) respectively provide the launch files and configuration files for other datasets in the paper. If you want to run AirSLAM with your own dataset, you need to create your own camera file, configuration file, and launch file. 


## :writing_hand: TODO List

- [x] Initial release. :rocket:
- [ ] Support more GPUs and development environments
- [ ] Support SuperGlue as the feature matcher
- [ ] Optimize the TensorRT acceleration of PLNet


## :pencil: Citation
```bibtex
@article{xu2024airslam,
  title = {{AirSLAM}: An Efficient and Illumination-Robust Point-Line Visual SLAM System},
  author = {Xu, Kuan and Hao, Yuefan and Yuan, Shenghai and Wang, Chen and Xie, Lihua},
  journal = {arXiv preprint arXiv:2408.03520},
  year = {2024},
  url = {https://arxiv.org/abs/2408.03520},
  code = {https://github.com/sair-lab/AirSLAM},
}

@inproceedings{xu2023airvo,
  title = {{AirVO}: An Illumination-Robust Point-Line Visual Odometry},
  author = {Xu, Kuan and Hao, Yuefan and Yuan, Shenghai and Wang, Chen and Xie, Lihua},
  booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year = {2023},
  url = {https://arxiv.org/abs/2212.07595},
  code = {https://github.com/sair-lab/AirVO},
  video = {https://youtu.be/YfOCLll_PfU},
}
```
