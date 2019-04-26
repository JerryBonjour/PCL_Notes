# pcl_ground_clip
ray filter 基于射线的地面分割方法

- 启动方法：roslaunch pcl_ground_clip ground_clip.launch
- 存在问题
    - 存在少量噪点，无法彻底过滤出地面
    - 非地面点容易被误分类，造成非地面点缺失
    - 目标接近激光雷达盲区，会出现误分割
