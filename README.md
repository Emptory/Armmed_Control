# Armmed Control

## OCS2 NMPC for 7-DOF Manipulator

*   Ubuntu 20.04
*   ROS Noetic
*   OCS2 Framework
*   Pinocchio

---

本项目实现了一个基于 **OCS2 (Optimal Control for Switched Systems)** 框架的非线性模型预测控制器 (NMPC)，用于控制 7 自由度机械臂（KUKA LBR iiwa）。

项目利用 **Pinocchio** 库进行刚体动力学计算 (RNEA)，并结合 **Gazebo** 进行物理仿真。控制器能够实现高频率 (50Hz+) 的实时最优控制，完成末端轨迹追踪或重力补偿任务。

---

## 视频

https://github.com/user-attachments/assets/8196640e-cda4-47c5-b20e-a1f3e5d8cfbc