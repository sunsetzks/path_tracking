# 1. 单选题：伺服模式区别（servo_mode）

题目：在伺服规划系统中，`servo_mode` 常见取值为 `0` 和 `1`。下列哪一项最准确地描述了 `servo_mode=0` 与 `servo_mode=1` 的主要区别？

A. `servo_mode=0` 表示自适应模式，使用较短的预留距离以提高效率；`servo_mode=1` 表示普通模式，使用标准预留距离。

B. `servo_mode=0` 表示普通模式，使用标准预留距离；`servo_mode=1` 表示自适应模式，通常使用较短的预留距离以提高进入货架效率。

C. 两者没有功能差别，仅为兼容历史配置的别名，实际行为由其他参数决定。

D. `servo_mode=0` 用于退出货架（direction=-1），`servo_mode=1` 用于进入货架（direction=1）。

正确答案：B

简要解析：根据系统配置约定，`servo_mode=0` 通常为普通模式（标准预留距离），而 `servo_mode=1` 表示自适应或高效模式，会使用较短的预留距离和更激进的对齐策略，从而提高进入货架的效率（但可能需要更高的定位与控制精度）。

## 2. 单选题：伺服规划系统的中心配置文件通常位于项目中哪个文件？

A. `config.json`

B. `pp_shelf_config.json`

C. `servo_config.ini`

D. `settings.toml`

正确答案：B

## 3. 单选题：伺服规划系统中，伺服日志文件的名字是什么？

A) servo.log  
B) pp_shelf.log  
C) planning.log  
D) agv.log  

正确答案：B

简要解析：根据伺服规划系统用户培训教程，日志文件路径为 `/home/agv/log/vn_path_planning/pp_shelf.log`，因此日志文件名为 `pp_shelf.log`。

## 4. 单选题：可视化工具有哪些

题目：在路径跟踪项目中，主要使用的可视化工具有哪些？

A. 仅Matplotlib  
B. IMGUI 和 Foxglove  
C. Foxglove 和 Plotly  
D. Matplotlib、Foxglove 和 matplotlib_fallback_visualization  

正确答案：B

简要解析：项目使用Matplotlib进行标准二维可视化（带交互控件），Foxglove进行实时3D可视化（带WebSocket服务器和MCAP记录）。matplotlib_fallback_visualization是Foxglove不可用时的Matplotlib后备方案，但不是独立工具。

## 5. 单选题：侧边框的参数有哪些

题目：side_box_y_width 的含义是什么？

A. 侧边框在前进方向的宽度  
B. 侧边框在左右方向的厚度  
C. 左侧边框相对托盘的偏移  
D. 右侧边框相对托盘的偏移  

正确答案：B

简要解析：side_box_y_width 定义了侧边框在左右方向的厚度，即货架侧板的实际物理厚度。

## 6. 单选题：机器人的参数有哪些

题目：robot_fork_length 的含义是什么？

A. 机器人车体后端X坐标  
B. 货叉长度  
C. 机器人车体前端X坐标  
D. 前方安全余量  

正确答案：B

简要解析：robot_fork_length 定义了货叉的长度（X方向），即从车体前端到货叉尖端的距离。

## 7. 单选题：托盘的参数有哪些

题目：tray_hole_center_ys 的含义是什么？

A. 托盘在前进方向的长度  
B. 托盘孔洞的Y坐标中心点数组  
C. 托盘在左右方向的宽度  
D. 货叉与孔洞的最小间距  

正确答案：B

简要解析：tray_hole_center_ys 定义了托盘孔洞的Y坐标中心点数组，用于确定货叉插入点的位置。

## 8. 单选题：托盘余量是哪个参数

题目：托盘余量是哪个参数？

A. tray_x_length  
B. tray_y_width  
C. tray_hole_fork_min_gap  
D. tray_goal_offset_x  

正确答案：C

简要解析：tray_hole_fork_min_gap 定义了货叉与孔洞的最小间距，即托盘余量，用于确保货叉安全插入。

## 9. 单选题：MCAP文件打开工具

题目：MCAP文件用什么打开？

A. Foxglove Studio  
B. Matplotlib  
C. Python脚本  
D. 文本编辑器  

正确答案：A

简要解析：MCAP是Foxglove的数据记录格式，用于实时可视化和回放机器人数据。Foxglove Studio是专门用于打开和分析MCAP文件的工具，提供3D可视化和数据回放功能。

## 10. 单选题：有几个侧边框

题目：货架通道有几个侧边框？

A. 1个  
B. 2个  
C. 3个  
D. 4个  

正确答案：B

简要解析：侧边框是指货架通道两侧的固定结构（货架侧板），通常有左右两个侧边框，用于构成机器人运动的物理边界。

## 11. 单选题：托盘宽度是哪个参数

题目：托盘宽度是哪个参数？

A. tray_x_length  
B. tray_y_width  
C. tray_hole_fork_min_gap  
D. tray_goal_offset_x  

正确答案：B

简要解析：tray_y_width 定义了托盘在左右方向的宽度，即托盘的实际物理宽度。

## 12. 多选题：系统使用的坐标系（可多选）

题目：下面哪些是伺服规划系统中常用的坐标系？（可多选）

- A. `map`（地图坐标系）
- B. `odom`（里程计坐标系）
- C. `robot`（机器人坐标系）
- D. `park_pose`（停车位姿坐标系）
- E. `border`（边界坐标系）
- F. `camera`（相机局部像素坐标系）

正确答案：A, B, C, D, E

简要说明：系统核心使用 `map`、`odom`、`robot`、`park_pose` 和 `border` 五个坐标系来分别表示全局地图、里程计、机器人自身、停车位姿和工作空间边界；`camera` 像素坐标系可能存在于感知模块，但不属于伺服规划的主要参考坐标系。


