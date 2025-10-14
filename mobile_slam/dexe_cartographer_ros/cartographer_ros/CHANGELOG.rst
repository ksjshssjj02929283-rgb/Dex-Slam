^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cartographer_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
<<<<<<< HEAD

2.0.9001 (2023-04-13)
---------------------
* fix cmake to prevent multiple definitions (`#63 <https://github.com/ros2/cartographer_ros/issues/63>`_)
* Contributors: Wolf Vollprecht

2.0.9000 (2022-03-31)
---------------------
* Update to latest upstream, as well as new port to ROS 2.

1.0.0 (2018-06-01)
------------------
* https://github.com/googlecartographer/cartographer_ros/compare/0.3.0...1.0.0

0.3.0 (2017-11-23)
------------------
* https://github.com/googlecartographer/cartographer_ros/compare/0.2.0...0.3.0

0.2.0 (2017-06-19)
------------------
* https://github.com/googlecartographer/cartographer_ros/compare/0.1.0...0.2.0

0.1.0 (2017-05-18)
------------------
* First unstable development release
=======
2.0.6 (2024-09-18)
----------------------
* 增加loam线特征，提高点云匹配效率

2.0.5 (2024-09-04)
----------------------
* 优化pose initializer的逻辑，确保初始定位的正确性

2.0.0-4 (2024-09-03)
----------------------
* 去掉定位中的gicp，定位中增加全局点云约束

2.0.0-3 (2024-09-03)
----------------------
* 去掉建图中的gicp，local建图效果偶尔会飘，不影响整体建图效果，同时提升速度

2.0.0-2 (2024-09-03)
----------------------
* 后端constrain builder中增加点云约束,建图效果更加稳定
>>>>>>> 0c267b7 (修改目录层级，删除不必要的文件)
