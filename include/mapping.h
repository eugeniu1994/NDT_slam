
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <time.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/registration/ndt.h>

using namespace std;

class mapping{
    public:
        mapping(ros::NodeHandle nh, const string laser_topic_name, int _max_iter, double _ndt_step_size, double _ndt_resolution, double _T_epsilon,
                double _voxel_leaf_size, double _scan_rate, double _min_range, double _max_range);
        ~mapping();
    private:

        ros::Subscriber cloud_sub;

        int max_iter; //number of iterations
        double ndt_resolution, ndt_step_size;
        double T_epsilon, voxel_leaf_size, scan_rate, min_range, max_range;
        bool use_imu, _incremental_voxel_update, is_first_map_;
        int first_scan_loaded;
        double scan_sift_min_add; //when to add a new keyframe
        double tf_x, tf_y, tf_z, tf_roll, tf_pitch, tf_yaw;
        Eigen::Matrix4f tf_btol, tf_ltob;

        ros::Publisher map_pub, pose_pub;
        geometry_msgs::PoseStamped curr_pose_msg;
        tf::TransformBroadcaster tf_broadcaster;

        //pose
        struct T{
            double X, Y, Z, roll, pitch, yaw;
        };

        struct T pose_curr, pose_imu_current, pose_prev;

        pcl::PointCloud<pcl::PointXYZI> map;
        pcl::VoxelGrid<pcl::PointXYZI> voxel_map_filter;
        pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> NDT;

        std::ofstream ofs;
        std::string filename;

        void IMU_callback(const sensor_msgs::Imu::Ptr& msg);
        void IMU_process(ros::Time curr_time);
        void LaserScan_callback_NDT(const sensor_msgs::PointCloud2::ConstPtr& msg);
        void LaserScan_callback_ICP_NDT(const sensor_msgs::PointCloud2::ConstPtr& msg);

};