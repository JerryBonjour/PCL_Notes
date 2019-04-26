# pcl_icp
icp 迭代最近点法配准
```所有的案例pcd、ply文件均在cmake-build-debug里面,不要轻易删除.
main函数只有一个,需要跑icp1案例的时候,注释掉其他案例的main函数.
```
- icp1:两幅图像通过icp求变换矩阵

- icp2:icp对点云数据配准计算
- icp3
    - 加载点云，进行刚体变换，使用icp算法将变换后的点云与原来的点云对齐
    - 启动方法：./pcl_icp bunny.ply 
- pairwise:匹配多副点云