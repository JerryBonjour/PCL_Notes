# pcl_ground_plane_fitting
Ground Plane Fitting 地面平面拟合方法
- 特点
    - 现实的地面并非完美的平面,距离较远时存在一定的测量噪声,这种坡度变化不能视作非地面
    - 在可靠性及精度方面，此法均优于基于射线的地面切割方法
    - 出自论文：Fast segmentation of 3D point clouds: A paradigm on LiDAR data for autonomous vehicle applications

- 算法原理概述
```
沿x方向将空间分割成若干个子平面, 对每个子平面使用GPF算法,因此可以处理陡坡
选取种子点集，建立地面初始平面模型
计算点云中每一个点到该平面的正交投影的距离,与预先设定的阀值进行比较,判定是否为地面点
分类后的地面点作为新的种子点,进一步迭代优化
```
- 启动方式:`roslaunch pcl_ground__plane_fitting plane_fitting.launch`




