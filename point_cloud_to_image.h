#pragma once

#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>

#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>

template <typename PointT>
class PointCloud2Image {
 public:
  PointCloud2Image(const double scale, const pcl::PointCloud<PointT>& cloud)
      : scale_(scale) {
    pcl::getMinMax3D(cloud, min_pt_, max_pt_);

    // image height and width
    // +1 to cover min_pt_ and max_pt_
    height_ = (max_pt_.x - min_pt_.x) / scale + 1;
    width_ = (max_pt_.y - min_pt_.y) / scale + 1;

    T_cloud_image_.setIdentity();
    T_cloud_image_.topLeftCorner(3, 3) << 0, -1, 0, -1, 0, 0, 0, 0, -1;
    T_cloud_image_(0, 3) = max_pt_.y;
    T_cloud_image_(1, 3) = max_pt_.x;
    T_cloud_image_(2, 3) = 0;
  }

  cv::Mat ToImage(const pcl::PointCloud<PointT>& source_cloud) {
    cv::Mat img(height_, width_, CV_8UC1, cv::Scalar(0));
    pcl::PointCloud<PointT> transformed_cloud;
    pcl::transformPointCloud(source_cloud, transformed_cloud, T_cloud_image_);
    for (auto& pt : transformed_cloud.points) {
      int i = pt.x / scale_;
      int j = pt.y / scale_;
      img.at<uchar>(j, i) = 255;
    }
    return img;
  }

 public:
  // 由点云坐标转图像坐标
  Eigen::Matrix4d T_cloud_image_;

  const double scale_;
  // 坐标原点
  Eigen::Matrix4d origin_;
  PointT min_pt_, max_pt_;
  int width_, height_;
};