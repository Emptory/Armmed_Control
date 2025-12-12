# Armmed Control

## OCS2 NMPC for 7-DOF Manipulator

*   Ubuntu 20.04
*   ROS Noetic
*   OCS2
*   Pinocchio
*   legged_control

---

本项目在 **OCS2 (Optimal Control for Switched Systems)** 框架下搭建了一个针对 7 自由度机械臂（KUKA LBR iiwa）的非线性模型预测控制器 (NMPC)。

本项目主要基于legged_control开源项目的manipulator实例修改，最好安装在legged_control的src目录下。

---

### 实现思路

本项目采用 **前馈 + PD 反馈** 的力矩控制策略。

$$ \tau_{cmd} = \tau_{ff}(q_{ref}, \dot{q}_{ref}, \ddot{q}_{ref}) + K_p(q_{ref} - q_{act}) + K_d(\dot{q}_{ref} - \dot{q}_{act}) $$

* **动力学前馈 ($\tau_{ff}$)**: 使用 **Pinocchio** 库的 RNEA (Recursive Newton-Euler Algorithm) 算法计算。

  输入为当前实际状态 ($q_{act}, \dot{q}_{act}$) 和 MPC 规划出的期望加速度 ($\ddot{q}_{ref}$)。
* **反馈修正**: 使用 PD 控制器补偿模型误差。

---

### 基于 legged_control 的适配

本项目代码主要参考并修改自 `legged_control` 仓库中的 `mobile_manipulator` 示例。原示例是针对四足机器人搭载机械臂的移动操作平台设计的。

1.  **动力学模型配置**：
    *   只保留了源代码中固定基座 (Fixed Base) 动力学配置，适配7 自由度机械臂。
    *   调整了 Pinocchio 的模型映射（Mapping），移除了与底座相关的状态维度。
2.  **任务空间约束**：
    *   修改了末端执行器（End-Effector）的约束定义，去除了与移动底座耦合的部分，仅保留机械臂末端的 Pose 约束。
    *   调整了 Cost Function 中的权重矩阵（Q, R），针对机械臂的关节力矩范围和运动特性进行了重新整定。
3.  **工程结构**：
    *   复用了 `legged_control` 的一些接口。

----

### 文件结构

```
legged_ws/
├── src/
│   ├── arm_control/
│   │   ├── launch/
│   │   │   └── arm_control.launch
│   │   ├── config/
│   │   │   ├── arm_controller.yaml
│   │   │   └── arm_slq_config.yaml
│   │   ├── include/
│   │   └── src/
    		├── ArmController.cpp
    	    ├── FactoryFunctions.cpp
    	    ├── MobileManipulatorInterface.cpp
    	    ├── MobileManipulatorPinocchioMapping.cpp
    	    ├── MobileManipulatorPreComputation.cpp
    	    ├── SineReferenceManager.cpp
     	    ├── constraint/EndEffectorConstraint.cpp
     	    └── dynamics/DefaultManipulatorDynamics.cpp
│   ├── arm_description/
│   │   ├── urdf/
│   │   │   ├── lbr_iiwa_14_r820.urdf
│   │   │   └── lbr_iiwa_14_r820.xacro
│   │   ├── meshes/
│   │   └── launch/gazebo_kuka.launch
│   └── arm_gazebo/
        └── launch/arm_world.launch
```

简要说明：
- arm_control：核心控制代码（OCS2 接口、参考管理、代价/约束、动力学封装）。
  - arm_controller.yaml
    - MPC 运行频率：mpc_rate
    - SineReference ：目标位置 base_pos、振幅 amplitude_x/ amplitude_z、频率 frequency 等。
    - kp,kd。
  - MobileManipulatorPinocchioMapping.cpp
    - Pinocchio映射
  - MobileManipulatorPreComputation.cpp
    - 预计算，OCS2提前计算与重用某些动力学/雅可比相关项所要求的
  - SineReferenceManager.cpp
    - 使用ocs2::TargetTrajectories生成正弦波
  - EndEffectorConstraint.cpp
    - 末端执行器的软约束实现，把末端误差转换为 cost/constraint 项。
  - DefaultManipulatorDynamics.cpp
    - 固定基座机械臂的动力学实现。
  - MobileManipulatorInterface.cpp
    - 配置整个OCS2的最优控制问题
  - ArmController.cpp
    - 控制器，负责根据MPC输出的动力学预测的序列插值之后，转成力矩，
- arm_description：机械臂 URDF / mesh与 Gazebo 加载配置。
- arm_gazebo：仿真场景，启动文件。

---

### 视频&图片

展示效果

https://github.com/user-attachments/assets/48769388-ea3e-4778-ba22-d5b5bea2b204

![Image](https://github.com/user-attachments/assets/46f176bd-bfdb-4dfd-9512-1680668825d7)