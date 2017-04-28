This is the webpage of the open-source packages of our paper submitted to ICRA2016. Our implementation of dense tracking is built on the top of the open-source and vectorized implementation of [1]. Our package is compatible with the standard driver of VI-Sensor and ROS version of indigo. OpenCV is needed. ROS packages related to VI-sensor are needed, too. (https://github.com/ethz-asl/libvisensor and https://github.com/ethz-asl/visensor_node.git)

The high resolution video of our submitted paper is: 
https://onedrive.live.com/redir?resid=907AC500FCC6D19C!256&authkey=!AFx_upC8zphx44w&ithint=video%2cmp4


We have uploaded the experimental data shown in the video. All the raw data, estimatior output, UKF smoothing output, control command and etc. are included in the rosbags. To download it, please visit,
https://www.dropbox.com/s/ebj3438g22wl8yl/icra_bags.zip?dl=0


If you use our code, please cite our paper

1. "Aggresive Quadrotor Flight Using Dense Visual-Inertial Fusion", in Proc. of the IEEE Intl. Conf. on Robot. and Autom., 2016. 

2. "Dense Visual-Inertial Odometry for Tracking of Aggressive Motions", in Proc. of the IEEE International Conference on Robotics and Biomimetics 2015.


For more questions, please contact ylingaa at connect dot ust dot hk .




[1] LSD-SLAM: Large-Scale Direct Monocular SLAM (J. Engel, T. Schöps, D. Cremers), In European Conference on Computer Vision (ECCV), 2014.
