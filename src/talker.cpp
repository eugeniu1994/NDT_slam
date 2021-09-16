#include "mapping.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapping");

  string laser_topic_name  = "/lidar_VLS128_Center_13202201895611/velodyne_points";
  int _max_iter;
  double _ndt_step_size, _ndt_resolution, _T_epsilon, _voxel_leaf_size, _scan_rate, _min_range, _max_range;

  ros::NodeHandle nh;
  ros::param::get("~max_iter", _max_iter);
  ros::param::get("~ndt_step_size", _ndt_step_size);
  ros::param::get("~ndt_resolution", _ndt_resolution);
  ros::param::get("~T_epsilon", _T_epsilon);
  ros::param::get("~voxel_leaf_size", _voxel_leaf_size);
  ros::param::get("~scan_rate", _scan_rate);
  ros::param::get("~min_range", _min_range);
  ros::param::get("~max_range", _max_range);

  cout<<"_max_iter "<<_max_iter<<" "<<"_ndt_step_size "<<_ndt_step_size<<" "<<"_ndt_resolution "<<_ndt_resolution<<" "<<"_T_epsilon "<<_T_epsilon<<" "<<"_voxel_leaf_size "<<_voxel_leaf_size<<" "<<"_scan_rate "<<_scan_rate<<std::endl;
  cout<<"_min_range "<<_min_range<<" _max_range "<<_max_range<<std::endl;

  //_max_iter = 15; ;
  //_ndt_step_size = 0.1; _ndt_resolution = 7.0; _T_epsilon = 0.01;
  //_voxel_leaf_size = 3.0; _scan_rate = 10.0; _min_range = 7.0; _max_range = 50;

  mapping NDT(nh, laser_topic_name, _max_iter, _ndt_step_size, _ndt_resolution, _T_epsilon,
                _voxel_leaf_size, _scan_rate, _min_range, _max_range);

  ros::spin();
  return 0;
};

