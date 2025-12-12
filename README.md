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

---

### 视频

https://github.com/user-attachments/assets/48769388-ea3e-4778-ba22-d5b5bea2b204

![Image](https://github.com/user-attachments/assets/46f176bd-bfdb-4dfd-9512-1680668825d7)