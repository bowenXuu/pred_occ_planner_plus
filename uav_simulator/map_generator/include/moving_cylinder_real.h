/**
 * @file moving_cylinder.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-07-07
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __MOVING_CYLINDER_H__
#define __MOVING_CYLINDER_H__

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Eigen>
#include <random>

namespace dynamic_map_objects {

class MovingCylinder {
 private:
  std::uniform_real_distribution<double> _rand_x, _rand_y, _rand_w, _rand_h, _rand_v;
  double                                 _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h, _v_h;
  double                                 _resolution;
  int                                    _mode = 0;

 public:
  pcl::PointCloud<pcl::PointXYZ> _cloud;
  double                         x;   // x coordinate of the cylinder center
  double                         y;   // y coordinate of the cylinder center
  double                         w;   // width
  double                         h;   // height
  double                         vx;  // x velocity
  double                         vy;  // y velocity
  MovingCylinder(double                      x_l,
                 double                      x_h,
                 double                      y_l,
                 double                      y_h,
                 double                      w_l,
                 double                      w_h,
                 double                      h_l,
                 double                      h_h,
                 double                      v_h,
                 std::default_random_engine &eng,
                 double                      _resolution,
                 double                      obs_x,
                 double                      obs_y);
  ~MovingCylinder() {}
  void setMode(int m);
  void update(double obs_x, double obs_y);
};

MovingCylinder::MovingCylinder(double                      x_l,
                               double                      x_h,
                               double                      y_l,
                               double                      y_h,
                               double                      w_l,
                               double                      w_h,
                               double                      h_l,
                               double                      h_h,
                               double                      v_h,
                               std::default_random_engine &eng,
                               double                      resolution,
                               double                      obs_x,
                               double                      obs_y)
    : _x_l(x_l)
    , _x_h(x_h)
    , _y_l(y_l)
    , _y_h(y_h)
    , _w_l(w_l)
    , _w_h(w_h)
    , _h_l(h_l)
    , _h_h(h_h)
    , _v_h(v_h) {
  _rand_x     = std::uniform_real_distribution<double>(x_l, x_h);
  _rand_y     = std::uniform_real_distribution<double>(y_l, y_h);
  _rand_w     = std::uniform_real_distribution<double>(w_l, w_h);
  _rand_h     = std::uniform_real_distribution<double>(h_l, h_h);
  _rand_v     = std::uniform_real_distribution<double>(-v_h, v_h);
  _resolution = resolution;

  // genrate random 2D position, width, height
  x = obs_x;
  y = obs_y;
  h = 2.000;
  w = 0.300;

  // generate random veocity
  vx = 0;
  vy = 0;

  int heightNum = ceil(h / _resolution);
  int widNum    = ceil(w / _resolution);

  // random point in grid scale
  x = floor(x / _resolution) * _resolution + _resolution / 2.0;
  y = floor(y / _resolution) * _resolution + _resolution / 2.0;

  // generate point cloud
  _cloud.points.resize(0);
  _cloud.width  = 0;
  _cloud.height = 0;

  pcl::PointXYZ pt(x, y, h);
  _cloud.points.push_back(pt);

  for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
    for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
      if ((r * r + s * s) > (widNum * widNum / 4.0)) {
        continue;
      }
      for (int t = -2.0; t < heightNum; t++) {
        pt.x = x + (r + 0.5) * _resolution + 1e-2;
        pt.y = y + (s + 0.5) * _resolution + 1e-2;
        pt.z = (t + 0.5) * _resolution + 1e-2;
        _cloud.points.push_back(pt);
      }
    }

  _cloud.width    = _cloud.points.size();
  _cloud.height   = 1;
  _cloud.is_dense = true;
};  // namespace dynamic_map_objects

void MovingCylinder::setMode(int m) {
  // generate random veocity
  if (m == 0) {
    ;
  } else if (m == 1) {
    vy = 0;
  } else if (m == 2) {
    vx = 0;
  } else {
    vx = 0;
    vy = 0;
  }
}

void MovingCylinder::update(double obs_x, double obs_y) {
  vx = obs_x-x;
  vy = obs_y-y;
  x = obs_x;
  y = obs_y;
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << vx, vy, 0;
  pcl::transformPointCloud(_cloud, _cloud, transform);
}

}  // namespace dynamic_map_objects

#endif  // __MOVING_CYLINDER_H__
