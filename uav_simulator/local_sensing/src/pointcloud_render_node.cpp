  #include <math.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <pcl/search/impl/kdtree.hpp>
#include <vector>

using namespace std;
using namespace Eigen;

ros::Publisher pub_cloud;

sensor_msgs::PointCloud2 local_map_pcl;
sensor_msgs::PointCloud2 local_depth_pcl;

ros::Subscriber odom_sub;
ros::Subscriber global_map_sub, local_map_sub;

ros::Timer local_sensing_timer;

bool has_global_map(false);
bool has_local_map(false);
bool has_odom(false);
bool is_camera_frame(false);
bool is_dynamic_map(false);

nav_msgs::Odometry _odom;

double fov_width  = 43.5; /* default 43.5 degree, from realsense 435D */
double fov_height = 29.0; /* default 29.0 degree, from realsense 435D */
double sensing_horizon, sensing_rate, estimation_rate;
double _x_size, _y_size, _z_size;
double _gl_xl, _gl_yl, _gl_zl;
double _resolution, _inv_resolution;
int    _GLX_SIZE, _GLY_SIZE, _GLZ_SIZE;

ros::Time last_odom_stamp = ros::TIME_MAX;

inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i& index) {
  Eigen::Vector3d pt;
  pt(0) = ((double)index(0) + 0.5) * _resolution + _gl_xl;
  pt(1) = ((double)index(1) + 0.5) * _resolution + _gl_yl;
  pt(2) = ((double)index(2) + 0.5) * _resolution + _gl_zl;

  return pt;
};

inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d& pt) {
  Eigen::Vector3i idx;
  idx(0) = std::min(std::max(int((pt(0) - _gl_xl) * _inv_resolution), 0), _GLX_SIZE - 1);
  idx(1) = std::min(std::max(int((pt(1) - _gl_yl) * _inv_resolution), 0), _GLY_SIZE - 1);
  idx(2) = std::min(std::max(int((pt(2) - _gl_zl) * _inv_resolution), 0), _GLZ_SIZE - 1);

  return idx;
};

void rcvOdometryCallbck(const nav_msgs::Odometry& odom) {
  /*if(!has_global_map)
    return;*/
  has_odom = true;
  _odom    = odom;
}

pcl::PointCloud<pcl::PointXYZ>                   _cloud_all_map, _local_map, _visible_map;
pcl::VoxelGrid<pcl::PointXYZ>                    _voxel_sampler;
pcl::VoxelGridOcclusionEstimation<pcl::PointXYZ> _occlusion_estimator;
sensor_msgs::PointCloud2                         _local_map_pcd;

pcl::search::KdTree<pcl::PointXYZ> _kdtreeLocalMap;
vector<int>                        _pointIdxRadiusSearch;
vector<float>                      _pointRadiusSquaredDistance;
float                              _leaf_size = 0.1;

void rcvGlobalPointCloudCallBack(const sensor_msgs::PointCloud2& pointcloud_map) {
  if (has_global_map && !is_dynamic_map) return;

  ROS_INFO("Global Pointcloud received..");

  pcl::PointCloud<pcl::PointXYZ> cloud_input;
  pcl::fromROSMsg(pointcloud_map, cloud_input);

  _voxel_sampler.setLeafSize(_leaf_size, _leaf_size, _leaf_size);
  _voxel_sampler.setInputCloud(cloud_input.makeShared());
  _voxel_sampler.filter(_cloud_all_map);

  _kdtreeLocalMap.setInputCloud(_cloud_all_map.makeShared());

  has_global_map = true;
}

void renderSensedPoints(const ros::TimerEvent& event) {
  if (!has_global_map || !has_odom) return;

  Eigen::Quaterniond q;
  q.x() = _odom.pose.pose.orientation.x;
  q.y() = _odom.pose.pose.orientation.y;
  q.z() = _odom.pose.pose.orientation.z;
  q.w() = _odom.pose.pose.orientation.w;

  Eigen::Matrix3d rot;
  rot                        = q;
  Eigen::Vector3d body_x_vec = rot.col(0); /* head direction in world frame */
  Eigen::Vector3d body_y_vec = rot.col(1);
  Eigen::Vector3d body_z_vec = rot.col(2);

  _local_map.points.clear();
  pcl::PointXYZ searchPoint(_odom.pose.pose.position.x, _odom.pose.pose.position.y,
                            _odom.pose.pose.position.z);
  _pointIdxRadiusSearch.clear();
  _pointRadiusSquaredDistance.clear();

  pcl::PointXYZ pt;
  if (_kdtreeLocalMap.radiusSearch(searchPoint, sensing_horizon, _pointIdxRadiusSearch,
                                   _pointRadiusSquaredDistance) > 0) {
    for (size_t i = 0; i < _pointIdxRadiusSearch.size(); ++i) {
      pt = _cloud_all_map.points[_pointIdxRadiusSearch[i]];

      if ((fabs(pt.z - _odom.pose.pose.position.z) / sensing_horizon) > tan(M_PI / 6.0)) continue;

      /* point in the world frame */
      Vector3d pt_world(pt.x - _odom.pose.pose.position.x, pt.y - _odom.pose.pose.position.y,
                      pt.z - _odom.pose.pose.position.z);
      Vector3d pt_body = rot.transpose() * pt_world; /* rotate the point cloud to body frame */

      /* remove points that are not in the sensing horizon */
      if (fabs(pt_body(2)) / sensing_horizon > tan(M_PI / 6.0)) continue;

      double x = pt_body(0);
      double y = pt_body(1);
      double z = pt_body(2);
      
      double body_w_cos = x / sqrt(x * x + y * y);
      double body_h_cos = x / sqrt(x * x + z * z);

      if (is_camera_frame) { /* if outputs point clouds in y-z-x camera frame */
        if (body_w_cos > cos(M_PI / 180 * fov_width) && body_h_cos > cos(M_PI / 180 * fov_height)) {
          pcl::PointXYZ pt_camera_frame(-pt_body[1], -pt_body[2], pt_body[0]);
          _local_map.points.push_back(pt_camera_frame);
        }
      } else { /* output point clouds in local frame */
        pcl::PointXYZ pt_local(pt_world[0], pt_world[1], pt_world[2]);
        _local_map.points.push_back(pt_local);
      }
    }
  } else {
    return;
  }

  _visible_map.points.clear();
  _occlusion_estimator.setLeafSize(_leaf_size, _leaf_size, _leaf_size);
  _occlusion_estimator.setInputCloud(_local_map.makeShared());
  _occlusion_estimator.initializeVoxelGrid();
  for (size_t i = 0; i < _local_map.points.size(); i++) {
    pcl::PointXYZ   pt      = _local_map.points[i];
    Eigen::Vector3i grid_pt = _occlusion_estimator.getGridCoordinates(pt.x, pt.y, pt.z);

    int occluded;
    int res = _occlusion_estimator.occlusionEstimation(occluded, grid_pt);
    if (occluded == 0) {
      _visible_map.points.push_back(pt);
    }
  }
  _visible_map.width    = _visible_map.points.size();
  _visible_map.height   = 1;
  _visible_map.is_dense = true;
  pcl::toROSMsg(_visible_map, _local_map_pcd);
  _local_map_pcd.header.frame_id = "world";
  _local_map_pcd.header.stamp = ros::Time::now();

  pub_cloud.publish(_local_map_pcd);
}

void rcvLocalPointCloudCallBack(const sensor_msgs::PointCloud2& pointcloud_map) {
  // do nothing, fix later
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcl_render");
  ros::NodeHandle nh("~");

  nh.getParam("is_camera_frame", is_camera_frame); /* output point clouds in y-z-x camera frame */
  nh.getParam("is_dynamic_map", is_dynamic_map);   /* if the map is dynamic */
  nh.getParam("sensing_horizon", sensing_horizon);
  nh.getParam("sensing_rate", sensing_rate);
  nh.getParam("estimation_rate", estimation_rate);
  nh.getParam("leaf_size", _leaf_size);
  nh.getParam("fov_width", fov_width);
  nh.getParam("fov_height", fov_height);

  nh.getParam("map/x_size", _x_size);
  nh.getParam("map/y_size", _y_size);
  nh.getParam("map/z_size", _z_size);

  // subscribe point cloud
  global_map_sub = nh.subscribe("global_map", 1, rcvGlobalPointCloudCallBack);
  local_map_sub  = nh.subscribe("local_map", 1, rcvLocalPointCloudCallBack);
  odom_sub       = nh.subscribe("odometry", 50, rcvOdometryCallbck);

  // publisher depth image and color image
  pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("pcl_render_node/cloud", 10);

  double sensing_duration = 1.0 / sensing_rate;

  local_sensing_timer = nh.createTimer(ros::Duration(sensing_duration), renderSensedPoints);

  _inv_resolution = 1.0 / _resolution;

  _gl_xl = -_x_size / 2.0;
  _gl_yl = -_y_size / 2.0;
  _gl_zl = 0.0;

  _GLX_SIZE = (int)(_x_size * _inv_resolution);
  _GLY_SIZE = (int)(_y_size * _inv_resolution);
  _GLZ_SIZE = (int)(_z_size * _inv_resolution);

  ros::Rate rate(100);
  bool      status = ros::ok();
  while (status) {
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }
}
