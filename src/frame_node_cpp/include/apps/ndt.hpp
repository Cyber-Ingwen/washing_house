#ifndef APPS_NDT_HPP_
#define APPS_NDT_HPP_

#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>