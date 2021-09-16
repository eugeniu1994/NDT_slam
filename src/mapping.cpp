#include "mapping.h"

using namespace std;

mapping::mapping(ros::NodeHandle nh, const string laser_topic_name, int _max_iter, double _ndt_step_size, double _ndt_resolution, double _T_epsilon,
                double _voxel_leaf_size, double _scan_rate, double _min_range, double _max_range){

     cloud_sub = nh.subscribe(laser_topic_name, 10000, &mapping::LaserScan_callback_NDT,this);
     //cloud_sub = nh.subscribe(laser_topic_name, 10000, &mapping::LaserScan_callback_ICP_NDT,this);

     map_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt_map", 5000);
     pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 5000);

     nh.param("max_iter", max_iter, _max_iter);
     nh.param("ndt_step_size", ndt_step_size, _ndt_step_size);
     nh.param("ndt_resolution", ndt_resolution, _ndt_resolution);
     nh.param("T_epsilon", T_epsilon, _T_epsilon);
     nh.param("voxel_leaf_size", voxel_leaf_size, _voxel_leaf_size);
     nh.param("scan_rate", scan_rate, _scan_rate);
     nh.param("min_range", min_range, _min_range);
     nh.param("max_range", max_range, _max_range);
     nh.param("use_imu", use_imu, false);  //dont have the IMU yet

     first_scan_loaded = 0;
     scan_sift_min_add = 2.0;     // m, add new keyframe
     tf_x = 0, tf_y = 0, tf_z = 0, tf_roll = 0, tf_pitch = 0, tf_yaw = 0;
     Eigen::Translation3f translation_btol(tf_x, tf_y, tf_z);
     Eigen::AngleAxisf rot_x_btol(tf_roll, Eigen::Vector3f::UnitX());
     Eigen::AngleAxisf rot_y_btol(tf_pitch, Eigen::Vector3f::UnitY());
     Eigen::AngleAxisf rot_z_btol(tf_yaw, Eigen::Vector3f::UnitZ());

     tf_btol = (translation_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();   // base to laser
     tf_ltob = tf_btol.inverse();

     map.header.frame_id = "map";

     pose_curr.X = pose_curr.Y = pose_curr.Z = 0.0;
     pose_curr.roll = pose_curr.pitch = pose_curr.yaw = 0.0;

     pose_prev.X = pose_prev.Y = pose_prev.Z = 0.0;
     pose_prev.roll = pose_prev.pitch = pose_prev.yaw = 0.0;

     voxel_map_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);  //cubes
     is_first_map_ = true;
     NDT.setTransformationEpsilon(T_epsilon);
     NDT.setStepSize(ndt_step_size);
     NDT.setResolution(ndt_resolution);
     NDT.setMaximumIterations(max_iter);
};

mapping::~mapping(){};

void mapping::LaserScan_callback_ICP_NDT(const sensor_msgs::PointCloud2::ConstPtr& msg){
    pcl::PointCloud<pcl::PointXYZI> tmp, scan;
    tf::Quaternion q;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());

    Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
    Eigen::Matrix4f t_base_link(Eigen::Matrix4f::Identity());
    tf::Transform transform;

    pcl::fromROSMsg(*msg, tmp);
    double r;
    Eigen::Vector3d point_pos;
    pcl::PointXYZI p;
    for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = tmp.begin(); item != tmp.end(); item++)
    {
        if(use_imu){  //
              // deskew(TODO:inplement of predicting pose by imu)
              point_pos.x() = (double)item->x;
              point_pos.y() = (double)item->y;
              point_pos.z() = (double)item->z;
              double s = scan_rate * (double(item->intensity) - int(item->intensity));

              point_pos.x() -= s * curr_pose_msg.pose.position.x;//current_pose_imu_
              point_pos.y() -= s * curr_pose_msg.pose.position.y;
              point_pos.z() -= s * curr_pose_msg.pose.position.z;

              Eigen::Quaterniond start_quat, end_quat, mid_quat;
              mid_quat.setIdentity();
              end_quat = Eigen::Quaterniond(
                curr_pose_msg.pose.orientation.w,
                curr_pose_msg.pose.orientation.x,
                curr_pose_msg.pose.orientation.y,
                curr_pose_msg.pose.orientation.z);
              start_quat = mid_quat.slerp(s, end_quat);

              point_pos = start_quat.conjugate() * start_quat * point_pos;

              point_pos.x() += curr_pose_msg.pose.position.x;
              point_pos.y() += curr_pose_msg.pose.position.y;
              point_pos.z() += curr_pose_msg.pose.position.z;

              p.x = point_pos.x();
              p.y = point_pos.y();
              p.z = point_pos.z();
        }
        else{ // no IMU
              p.x = (double)item->x;
              p.y = (double)item->y;
              p.z = (double)item->z;
        }
        p.intensity = (double)item->intensity;
        r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
        if (min_range < r && r < max_range)
        {
            scan.push_back(p);
        }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

    if (first_scan_loaded==0){
        pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol);  // The first frame is converted to base
        map += *transformed_scan_ptr;
        first_scan_loaded = 1;
    }

    //Downsampling
    voxel_map_filter.setInputCloud(scan_ptr);
    voxel_map_filter.filter(*filtered_scan_ptr);

    //cloud_src_o = filtered_scan_ptr
    //Calculate surface normal
    pcl::NormalEstimation<pcl::PointXYZI,pcl::Normal> ne_src;
    ne_src.setInputCloud(filtered_scan_ptr);
    pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree< pcl::PointXYZ>());
    ne_src.setSearchMethod(tree_src);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud< pcl::Normal>);//pcl::Normal is a point type, including curvature
    ne_src.setRadiusSearch(0.02);//Search the range of neighboring points
    ne_src.compute(*cloud_src_normals);



    NDT.setInputSource(filtered_scan_ptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));
    if (is_first_map_ == true){
        NDT.setInputTarget(map_ptr);
        is_first_map_ = false;
    }

    Eigen::Translation3f init_translation(pose_curr.X, pose_curr.Y, pose_curr.Z);
    Eigen::AngleAxisf init_rotation_x(pose_curr.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(pose_curr.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(pose_curr.yaw, Eigen::Vector3f::UnitZ());

    Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * tf_btol;
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    NDT.align(*output_cloud, init_guess);
    t_localizer = NDT.getFinalTransformation();

    t_base_link = t_localizer * tf_ltob;
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);

    tf::Matrix3x3 mat_b;
    mat_b.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
                   static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
                   static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
                   static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
                   static_cast<double>(t_base_link(2, 2)));

    //Update current pose
    pose_curr.X = t_base_link(0, 3);
    pose_curr.Y = t_base_link(1, 3);
    pose_curr.Z = t_base_link(2, 3);
    mat_b.getRPY(pose_curr.roll, pose_curr.pitch, pose_curr.yaw, 1);

    transform.setOrigin(tf::Vector3(pose_curr.X, pose_curr.Y, pose_curr.Z));
    q.setRPY(pose_curr.roll, pose_curr.pitch, pose_curr.yaw);
    transform.setRotation(q);

    tf_broadcaster.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "map", "base_link"));

    double shift = sqrt(pow(pose_curr.X - pose_prev.X, 2.0) + pow(pose_curr.Y - pose_prev.Y, 2.0));

    if (shift >= scan_sift_min_add)
    {
        map += *transformed_scan_ptr;
        pose_prev.X = pose_curr.X;
        pose_prev.Y = pose_curr.Y;
        pose_prev.Z = pose_curr.Z;
        pose_prev.roll = pose_curr.roll;
        pose_prev.pitch = pose_curr.pitch;
        pose_prev.yaw = pose_curr.yaw;
        NDT.setInputTarget(map_ptr);

        sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*map_ptr, *map_msg_ptr);
        map_pub.publish(*map_msg_ptr);
    }

    // Update the pose of the current frame
    curr_pose_msg.header.frame_id = "map";
    curr_pose_msg.header.stamp = msg->header.stamp;
    curr_pose_msg.pose.position.x = pose_curr.X;
    curr_pose_msg.pose.position.y = pose_curr.Y;
    curr_pose_msg.pose.position.z = pose_curr.Z;
    curr_pose_msg.pose.orientation.x = q.x();
    curr_pose_msg.pose.orientation.y = q.y();
    curr_pose_msg.pose.orientation.z = q.z();
    curr_pose_msg.pose.orientation.w = q.w();

    pose_pub.publish(curr_pose_msg);
    std::cout << "-----------------------------------------------------------------" << std::endl;
};


void mapping::LaserScan_callback_NDT(const sensor_msgs::PointCloud2::ConstPtr& msg){
    pcl::PointCloud<pcl::PointXYZI> tmp, scan;
    tf::Quaternion q;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());

    Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
    Eigen::Matrix4f t_base_link(Eigen::Matrix4f::Identity());
    tf::Transform transform;

    pcl::fromROSMsg(*msg, tmp);
    double r;
    Eigen::Vector3d point_pos;
    pcl::PointXYZI p;
    for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = tmp.begin(); item != tmp.end(); item++)
    {
        if(use_imu){  //
              // deskew(TODO:inplement of predicting pose by imu)
              point_pos.x() = (double)item->x;
              point_pos.y() = (double)item->y;
              point_pos.z() = (double)item->z;
              double s = scan_rate * (double(item->intensity) - int(item->intensity));

              point_pos.x() -= s * curr_pose_msg.pose.position.x;//current_pose_imu_
              point_pos.y() -= s * curr_pose_msg.pose.position.y;
              point_pos.z() -= s * curr_pose_msg.pose.position.z;

              Eigen::Quaterniond start_quat, end_quat, mid_quat;
              mid_quat.setIdentity();
              end_quat = Eigen::Quaterniond(
                curr_pose_msg.pose.orientation.w,
                curr_pose_msg.pose.orientation.x,
                curr_pose_msg.pose.orientation.y,
                curr_pose_msg.pose.orientation.z);
              start_quat = mid_quat.slerp(s, end_quat);

              point_pos = start_quat.conjugate() * start_quat * point_pos;

              point_pos.x() += curr_pose_msg.pose.position.x;
              point_pos.y() += curr_pose_msg.pose.position.y;
              point_pos.z() += curr_pose_msg.pose.position.z;

              p.x = point_pos.x();
              p.y = point_pos.y();
              p.z = point_pos.z();
        }
        else{ // no IMU
              p.x = (double)item->x;
              p.y = (double)item->y;
              p.z = (double)item->z;
        }
        p.intensity = (double)item->intensity;
        r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
        if (min_range < r && r < max_range)
        {
            scan.push_back(p);
        }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

    if (first_scan_loaded==0){
        pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol);  // The first frame is converted to base
        map += *transformed_scan_ptr;
        first_scan_loaded = 1;
    }

    //Downsampling
    voxel_map_filter.setInputCloud(scan_ptr);
    voxel_map_filter.filter(*filtered_scan_ptr);
    NDT.setInputSource(filtered_scan_ptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));
    if (is_first_map_ == true){
        NDT.setInputTarget(map_ptr);
        is_first_map_ = false;
    }

    Eigen::Translation3f init_translation(pose_curr.X, pose_curr.Y, pose_curr.Z);
    Eigen::AngleAxisf init_rotation_x(pose_curr.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(pose_curr.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(pose_curr.yaw, Eigen::Vector3f::UnitZ());

    Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * tf_btol;
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    NDT.align(*output_cloud, init_guess);
    t_localizer = NDT.getFinalTransformation();

    t_base_link = t_localizer * tf_ltob;
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);

    tf::Matrix3x3 mat_b;
    mat_b.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
                   static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
                   static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
                   static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
                   static_cast<double>(t_base_link(2, 2)));

    //Update current pose
    pose_curr.X = t_base_link(0, 3);
    pose_curr.Y = t_base_link(1, 3);
    pose_curr.Z = t_base_link(2, 3);
    mat_b.getRPY(pose_curr.roll, pose_curr.pitch, pose_curr.yaw, 1);

    transform.setOrigin(tf::Vector3(pose_curr.X, pose_curr.Y, pose_curr.Z));
    q.setRPY(pose_curr.roll, pose_curr.pitch, pose_curr.yaw);
    transform.setRotation(q);

    tf_broadcaster.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "map", "base_link"));

    double shift = sqrt(pow(pose_curr.X - pose_prev.X, 2.0) + pow(pose_curr.Y - pose_prev.Y, 2.0));

    if (shift >= scan_sift_min_add)
    {
        map += *transformed_scan_ptr;
        pose_prev.X = pose_curr.X;
        pose_prev.Y = pose_curr.Y;
        pose_prev.Z = pose_curr.Z;
        pose_prev.roll = pose_curr.roll;
        pose_prev.pitch = pose_curr.pitch;
        pose_prev.yaw = pose_curr.yaw;
        NDT.setInputTarget(map_ptr);

        sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*map_ptr, *map_msg_ptr);
        map_pub.publish(*map_msg_ptr);
    }

    // Update the pose of the current frame
    curr_pose_msg.header.frame_id = "map";
    curr_pose_msg.header.stamp = msg->header.stamp;
    curr_pose_msg.pose.position.x = pose_curr.X;
    curr_pose_msg.pose.position.y = pose_curr.Y;
    curr_pose_msg.pose.position.z = pose_curr.Z;
    curr_pose_msg.pose.orientation.x = q.x();
    curr_pose_msg.pose.orientation.y = q.y();
    curr_pose_msg.pose.orientation.z = q.z();
    curr_pose_msg.pose.orientation.w = q.w();

    pose_pub.publish(curr_pose_msg);
    std::cout << "-----------------------------------------------------------------" << std::endl;
};
