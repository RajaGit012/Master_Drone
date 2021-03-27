//
// Created by patrick on 7-3-19.
//

#ifndef INCLUDE_SPC_UAV_VISUAL_PCP_COMMON_H
#define INCLUDE_SPC_UAV_VISUAL_PCP_COMMON_H

#define PCL_NO_PRECOMPILE //Starting with PCL-1.7 needed before including any PCL headers including templated algorithms as well.

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

namespace pcp {

const float zOffSetp = 0.1; //absolute setpoint offset in z direction
const float zOffCam = 0.1; //absolute camera offset in z direction

struct PointXYZ{ //_PointXYZ{
  PCL_ADD_POINT4D;

  friend std::ostream& operator << (std::ostream& os, const PointXYZ& p) {
    os << "(" << p.x << "," << p.y << "," << p.z <<  ")";
    return (os);
  };
} EIGEN_ALIGN16;


typedef  pcl::PointXYZ Point;
//typedef pcl::PointXYZRGB Point;
//typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;

}


POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZ,//_PointXYZ,
                                     (float, x, x)
                                     (float, y, y)
                                     (float, z, z)
)


//struct EIGEN_ALIGN16 PointXYZRGBEnt : public _PointXYZRGBEnt
//{
//  inline PointXYZRGBEnt () {   } //TODO: remaining constructors etc. if above alone does not work
//}
//
//POINT_CLOUD_REGISTER_POINT_WRAPPER(pcp::PointXYZRGBEnt, pcp::_PointXYZRGBEnt)


#endif //INCLUDE_SPC_UAV_VISUAL_PCP_COMMON_H
