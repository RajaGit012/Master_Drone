//
// Created by patrick on 8-3-19.
//

#ifndef INCLUDE_SPC_UAV_VISUAL_POINTCLOUD_PROCESSOR_H
#define INCLUDE_SPC_UAV_VISUAL_POINTCLOUD_PROCESSOR_H

#include <spc_uav_visual/pcp_common.h>
#include <spc_uav_visual/cloud_segment.h>
#include <vector>

namespace pcp {

class PointCloudProcessor{
 public:
  //deep copys the points regarding the dimensions into segment point clouds
  PointCloudProcessor(PointCloud::Ptr input_cloud, Eigen::Vector2f cloud_center, Eigen::Vector2f cloud_dim);

  //deep copys the segments point clouds into a merged point cloud
  PointCloud::Ptr mergeClouds();

  //segments to investigate - numbered clockwise:
  CloudSegment seg0_, seg1_, seg2_, seg3_;

  //cloud center on x-y plane
  Eigen::Vector2f cloud_center_;
  //cloud dimensions on x-y plane
  Eigen::Vector2f cloud_dim_;

  std::vector<CloudSegment*> seg_ptrs_;

  //returns the ID of the segment with the lowest entropy (number of points)
  // or -1 not found/error
  int getLowEntSegID();

  PointCloud::Ptr getMergedCloud() {
    return merged_cloud_;
  }

 private:
  PointCloud::Ptr input_cloud_;

  PointCloud::Ptr merged_cloud_;



};

}

#endif //SPC_UAV_VISUAL_POINTCLOUD_PROCESSOR_H
