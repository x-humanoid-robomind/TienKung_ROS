# 天工通用人形机器人软件系统 🎯
本仓库提供 **天工通用人形机器人 Lite 版本的软件系统**，基于 **ROS** 框架开发，负责机器人的基础运动控制和硬件驱动。该系统包含机器人本体控制、机器人描述、遥控器通信等关键模块，支持机器人在各种应用场景中的使用与二次开发。

```
📦 TienKung_ROS （项目根目录）     
 ├── 📜 README.md                   # 总览  
 ├── 📂 lite/                     # Lite 版本软件系统   
 │   ├── 📜 README.md             # Lite 版本说明文档  
 │   ├── 📜 .deps.repos           # 依赖管理文件  
 │   ├── 📜 .gitignore            # Git 忽略规则  
 │   ├── 📜 .gitlab-ci.yml        # CI/CD 配置文件  
 │   ├── 📜 .catkin_workspace     # ROS 工作空间配置  
 │   ├── 📂 src/                  # 源代码文件夹  
 │   │   ├── 📂 robot_description # 机器人描述  
 │   │   ├── 📂 usb_sbus          # 遥控器通信  
 │   │   ├── 📂 body_control      # 本体控制  
 │   │   ├── 📂 fast_ros                  

```
