//
// Created by patrick on 7-3-19.
//

#ifndef INCLUDE_SPC_UAV_VISUAL_CLOUD_SEGMENT_H
#define INCLUDE_SPC_UAV_VISUAL_CLOUD_SEGMENT_H

#include <spc_uav_visual/pcp_common.h>

namespace pcp {

class CloudSegment {
 public:
  CloudSegment(PointCloud::Ptr input_cloud, float min_x, float max_x, float min_y, float max_y,
      uint8_t  r_i, uint8_t g_i, uint8_t b_i, float center_phi, unsigned int seg_id);

  //average squared euclidean distance of points to each other in segment
  float getSqEuEntropy() {
    return sq_eu_entropy_;
  }

  //get number of points in segment
  unsigned int getNPointsEntropy(){
    return segment_cloud_->points.size();
  }

  PointCloud::Ptr getCloud() {
    return segment_cloud_;
  }

  //get the ID (0-3) of the segment
  unsigned int getSegID(){
    return seg_id_;
  }

  //get angle determining the center of the segment in polar coordinates
  float getCenterPhi(){
    return center_phi_;
  }

  //sets pointers to neighbour segments on unit circle
  //neg_dir_ next segment in negative/CW direction on the unit circle
  //pos_dir_ next segment in positive/CCW direction on the unit circle
  void setNeighbours(CloudSegment* neg_dir, CloudSegment* pos_dir);

  //get next segment in positive/CCW direction on the unit circle
  CloudSegment* getPosDirSeg(){
    return pos_dir_;
  }

  //get next segment in negative/CW direction on the unit circle
  CloudSegment* getNegDirSeg(){
    return neg_dir_;
  }


 private:
  CloudSegment(){};
  float sq_eu_entropy_;
  unsigned int seg_id_;
  float center_phi_; //angle determining the center of the segment in polar coordinates
  CloudSegment* pos_dir_; //next segment in positive/CCW direction on the unit circle
  CloudSegment* neg_dir_; //next segment in negative/CW direction on the unit circle

  void segmentCloud();

  //boundaries of segment
  float min_x_, max_x_;
  float min_y_, max_y_;

  //identification color
  uint8_t  r_i_, g_i_, b_i_;

  PointCloud::Ptr whole_cloud_;

  PointCloud::Ptr segment_cloud_;

};

}


#endif //SPC_UAV_VISUAL_CLOUD_SEGMENT_H
