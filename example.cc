#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>

#include "point_cloud_to_image.h"

int main(int argc, char** argv) {
  // 检查命令行参数
  if (argc != 2) {
    std::cerr << "请提供一个PCD文件作为参数" << std::endl;
    return -1;
  }

  // 创建点云对象并加载PCD文件
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1) {
    std::cerr << "无法读取PCD文件: " << argv[1] << std::endl;
    return -1;
  }

  // 初始化最大点和最小点
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D(*cloud, minPt, maxPt);

  cloud->points.emplace_back(maxPt);
  cloud->points.emplace_back(minPt);
  PointCloud2Image<pcl::PointXYZ> pc2mat(0.3, *cloud);

  std::cout << "point size: " << cloud->size() << std::endl;
  // 输出最大点和最小点
  std::cout << "最小点: " << pc2mat.min_pt_ << std::endl;
  std::cout << "最大点: " << pc2mat.max_pt_ << std::endl;
  std::cout << "Transform: \n" << pc2mat.T_cloud_image_ << std::endl;
  for (auto& pt : cloud->points) {
    std::cout << "origin: " << pt.x << ", " << pt.y << ". image: ";
    Eigen::Vector4d tmp(pt.x, pt.y, 0, 1);
    Eigen::Vector4d tmp2 = pc2mat.T_cloud_image_ * tmp;
    std::cout << tmp2.x() << ", " << tmp2.y() << std::endl;
  }
  std::cout << "width / height: " << pc2mat.width_ << " / " << pc2mat.height_
            << std::endl;

  cv::Mat img = pc2mat.ToImage(*cloud);
  cv::imshow("test", img);

  pcl::visualization::PCLVisualizer viewer("3D Viewer");
  viewer.setBackgroundColor(0, 0, 0);
  viewer.addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer.addCoordinateSystem(1.0);
  viewer.initCameraParameters();

  // Main visualization loop
  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);
    cv::waitKey(1);
  }

  return 0;
}