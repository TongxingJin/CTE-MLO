#ifndef PREPROCESS_HPP
#define PREPROCESS_HPP
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include "common_lib.hpp"

using namespace std;

enum LID_TYPE{AVIA = 1, AVIA2, OUST64, VELO16, REALSENSE, ROBOSENSE};
enum TIME_UNIT{SEC = 0, MS = 1, US = 2, NS = 3};

namespace velodyne_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      float time;
      uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (uint16_t, ring, ring)
)

namespace robosense_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      double timestamp;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace robosense_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(robosense_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, timestamp, timestamp)
)

namespace ouster_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      uint32_t t;
      uint16_t reflectivity;
      uint16_t ambient;
      uint32_t range;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

class Preprocess
{
  public:
  Preprocess() {
    blind = 0.01;
    point_filter_num = 1;
  }
  ~Preprocess(){}
  
  void process(const livox_ros_driver2::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out, const int lidar_id) {  
    avia2_handler(msg, lidar_id);
    *pcl_out = pl_surf;
  }

  void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out, const int lidar_id) {  
    avia_handler(msg, lidar_id);
    *pcl_out = pl_surf;
  }
  // void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out, const int lidar_id);
  void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out, const int lidar_id) {
    switch (time_unit[lidar_id])
    {
      case SEC:
        time_unit_scale = 1.e3f;
        break;
      case MS:
        time_unit_scale = 1.f;
        break;
      case US:
        time_unit_scale = 1.e-3f;
        break;
      case NS:
        time_unit_scale = 1.e-6f;
        break;
      default:
        time_unit_scale = 1.f;
        break;
    }
    // std::cout << lidar_type[lidar_id] << " " << ROBOSENSE << std::endl;
    switch (lidar_type[lidar_id])
    {
    case OUST64:
      oust64_handler(msg, lidar_id);
      break;

    case VELO16:
      velodyne_handler(msg, lidar_id);
      break;
      
    case ROBOSENSE:
      robosense_handler(msg, lidar_id);
      break;

    case REALSENSE:
      realsense_handler(msg, lidar_id);
      break;
    
    default:
      printf("Error LiDAR Type");
      break;
    }
    *pcl_out = pl_surf;
  }
  PointCloudXYZI pl_surf, pl_full;
  float time_unit_scale;
  int point_filter_num;
  std::vector<int> lidar_type, time_unit;
  double blind;
    

  private:
  void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, const int lidar_id) {
    pl_surf.clear();
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.size();

    double time_stamp = msg->header.stamp.toSec();
    for (int i = 0; i < pl_orig.points.size(); i++)
    {
      if (i % point_filter_num != 0) continue;

      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      
      if (range < (blind * blind)) continue;
      
      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.curvature = pl_orig.points[i].t * time_unit_scale; // curvature unit: ms
      pl_surf.points.push_back(added_pt);
    }
  }
  void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, const int lidar_id) {
    pl_surf.clear(); 
    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize == 0) return;
    /*****************************************************************/   
    if (pl_orig.points[0].time==0) cout << "Wrong LiDAR Driver: no time stamp" << endl;
    for (int i = 0; i < plsize; i++) {
      PointType added_pt;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].time * time_unit_scale;  // curvature unit: ms

      if (i % point_filter_num == 0)
      {
        if(added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > (blind * blind))
        {
          pl_surf.points.push_back(added_pt);
        }
      }
    }
  }

  void avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg, const int lidar_id)
  {
    pl_surf.clear();
    pl_full.clear();
    int plsize = msg->point_num;

    pl_surf.reserve(plsize);
    pl_full.resize(plsize);

    uint valid_num = 0;
    for(uint i=1; i<plsize; i++) {
      if(((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)) {
        valid_num ++;
        if (valid_num % point_filter_num == 0) {
          pl_full[i].x = msg->points[i].x;
          pl_full[i].y = msg->points[i].y;
          pl_full[i].z = msg->points[i].z;
          pl_full[i].intensity = msg->points[i].reflectivity;
          pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms

          if(((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
              || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
              && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > (blind * blind))) {
            pl_surf.push_back(pl_full[i]);
          }
        }
      }
    }
  }


  void avia2_handler(const livox_ros_driver2::CustomMsg::ConstPtr &msg, const int lidar_id) {
    pl_surf.clear();
    pl_full.clear();
    int plsize = msg->point_num;

    pl_surf.reserve(plsize);
    pl_full.resize(plsize);

    uint valid_num = 0;
    for(uint i=1; i<plsize; i++) {
      if(((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)) {
        valid_num ++;
        if (valid_num % point_filter_num == 0) {
          pl_full[i].x = msg->points[i].x;
          pl_full[i].y = msg->points[i].y;
          pl_full[i].z = msg->points[i].z;
          pl_full[i].intensity = msg->points[i].reflectivity;
          pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms

          if(((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
              || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
              && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > (blind * blind))) {
                pl_full[i].normal_x = 0;
                pl_full[i].normal_y = 0;
                pl_full[i].normal_z = 0;
                pl_surf.push_back(pl_full[i]);
          }
        }
      }
    }
  }

  void robosense_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, const int lidar_id) {
    pl_surf.clear();
    pcl::PointCloud<robosense_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize == 0) return;

    /*** These variables only works when no point timestamps given ***/
    if (pl_orig.points[0].timestamp==0) cout << "Wrong LiDAR Driver: no time stamp" << endl;
    for (int i = 0; i < plsize; i++) {
      if (std::isnan(pl_orig.points[i].x) || std::isnan(pl_orig.points[i].y) || 
          std::isnan(pl_orig.points[i].z)) continue;        
      PointType added_pt;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = (pl_orig.points[i].timestamp - pl_orig.points[0].timestamp)*time_unit_scale;
      if (i % point_filter_num == 0)
      {
        if(added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > (blind * blind))
        {
          pl_surf.points.push_back(added_pt);
        }
      }
    }
  }

  void realsense_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, const int lidar_id) {
    pl_surf.clear();
    pcl::PointCloud<pcl::PointXYZRGB> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize == 0) return;

    /*** These variables only works when no point timestamps given ***/    
    for (int i = 0; i < plsize; i++) {
      if (std::isnan(pl_orig.points[i].x) || std::isnan(pl_orig.points[i].y) || 
          std::isnan(pl_orig.points[i].z)) continue;        
      PointType added_pt;
      added_pt.normal_x = pl_orig.points[i].b;
      added_pt.normal_y = pl_orig.points[i].g;
      added_pt.normal_z = pl_orig.points[i].r;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.curvature = 0.0; //Solid State

      if (i % point_filter_num == 0)
      {
        if(added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > (blind * blind))
        {
          pl_surf.points.push_back(added_pt);
        }
      }
    }
  }
};

/* Key of Hash Table */
class VOXEL_LOC
{
public:
  int64_t x, y, z;

  VOXEL_LOC(int64_t vx=0, int64_t vy=0, int64_t vz=0): x(vx), y(vy), z(vz){}

  bool operator== (const VOXEL_LOC &other) const
  {
    return (x==other.x && y==other.y && z==other.z);
  }
};

/* Hash value */
namespace std
{
  template<>
  struct hash<VOXEL_LOC>
  {
    size_t operator() (const VOXEL_LOC &s) const
    {
      using std::size_t; using std::hash;
      return ((hash<int64_t>()(s.x) ^ (hash<int64_t>()(s.y) << 1)) >> 1) ^ (hash<int64_t>()(s.z) << 1);
    }
  };
}

class OCTO_TREE {
public:
  std::vector<Eigen::Vector3d>* plvec_tran;
  std::vector<double> feat_eigen_ratio;
  double voxel_center[3]; // x, y, z
  double quater_length;
  OCTO_TREE* leaves[8];
  // bool is2opt;// correspond;
  int is2opt;// correspond;
  pcl::PointCloud<PointType> root_centors;
  Eigen::Vector3d plane_center;
  Eigen::Matrix3d plane_cov;
  int points_size;
  OCTO_TREE() {    
    for(int i=0; i<8; i++) leaves[i] = nullptr;
    plvec_tran = new std::vector<Eigen::Vector3d>();
    is2opt = 2; //0: Unupdate voxel, 1: update voxel, 2: new voxel
    plane_center.setZero();
    plane_cov.setZero();
    points_size = 0;
    feat_eigen_ratio.resize(2);
  }

  // Used by "recut"
  PointType calc_eigen() {
    Eigen::Matrix3d sum_ppt = (plane_cov + plane_center*plane_center.transpose())*points_size;
    Eigen::Vector3d sum_p = plane_center * points_size;
    for(uint j=0; j<plvec_tran->size(); j++) {
      sum_ppt += (*plvec_tran)[j] * (*plvec_tran)[j].transpose();
      sum_p += (*plvec_tran)[j];
    }
    points_size = points_size + plvec_tran->size();
    plane_center = sum_p/points_size;
    plane_cov = sum_ppt/points_size - plane_center*plane_center.transpose();
    
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(plane_cov);
    feat_eigen_ratio[0] = saes.eigenvalues()[0];
    feat_eigen_ratio[1] = saes.eigenvalues()[1];
    Eigen::Vector3d direct_vec = saes.eigenvectors().col(0);
    PointType ap_centor_direct;
    ap_centor_direct.x = plane_center.x();
    ap_centor_direct.y = plane_center.y();
    ap_centor_direct.z = plane_center.z();
    ap_centor_direct.normal_x = direct_vec.x();
    ap_centor_direct.normal_y = direct_vec.y();
    ap_centor_direct.normal_z = direct_vec.z();
    return ap_centor_direct;
  }

  void recut(int layer, int layer_max, pcl::PointCloud<PointType> &pl_feat_map, const int min_size) {
    if(plvec_tran->size()+points_size < min_size) return;

    PointType ap_centor_direct = calc_eigen(); // calculate eigenvalue ratio
    ap_centor_direct.intensity = layer;

    if((feat_eigen_ratio[1]/feat_eigen_ratio[0]) >= 6 || feat_eigen_ratio[0] < 0.05) {
      pl_feat_map.push_back(ap_centor_direct);
      if(plvec_tran != nullptr) std::vector<Eigen::Vector3d>().swap(*plvec_tran);
      return;
    }
    
    if(layer == layer_max) {
      if(plvec_tran != nullptr) std::vector<Eigen::Vector3d>().swap(*plvec_tran);
      return;
    }

    int leafnum;
    for(uint j=0; j<plvec_tran->size(); j++) {
      int xyz[3] = {0, 0, 0};
      for(uint k=0; k<3; k++) {
        if((*plvec_tran)[j][k] > voxel_center[k]) xyz[k] = 1;
      }
      leafnum = 4*xyz[0] + 2*xyz[1] + xyz[2];
      if(leaves[leafnum] == nullptr) {
        leaves[leafnum] = new OCTO_TREE();
        leaves[leafnum]->voxel_center[0] = voxel_center[0] + (2*xyz[0]-1)*quater_length;
        leaves[leafnum]->voxel_center[1] = voxel_center[1] + (2*xyz[1]-1)*quater_length;
        leaves[leafnum]->voxel_center[2] = voxel_center[2] + (2*xyz[2]-1)*quater_length;
        leaves[leafnum]->quater_length = quater_length / 2;
      }
      leaves[leafnum]->plvec_tran->push_back((*plvec_tran)[j]);
    }
    
    if(plvec_tran != nullptr) std::vector<Eigen::Vector3d>().swap(*plvec_tran);

    layer++;
    for(uint i=0; i<8; i++) {
      if(leaves[i] != nullptr) leaves[i]->recut(layer, layer_max, pl_feat_map, min_size);
    }
  }
};

void saveVoxel(OCTO_TREE* octo, visualization_msgs::MarkerArray& voxel_plane, uint& plane_id) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(octo->plane_cov);
  if (saes.eigenvalues()[1]/saes.eigenvalues()[0]>6.0 || saes.eigenvalues()[0]<0.05) {
    double trace = octo->plane_cov.diagonal().sum();
    trace = trace * (1.0 / 0.25);
    trace = pow(trace, 0.2);
    uint8_t r, g, b;
    mapJet(trace, 0, 1, r, g, b);
    Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
    visualization_msgs::Marker plane;
    plane.header.frame_id = "camera_init";
    plane.header.stamp = ros::Time();
    plane.ns = "voxel";
    plane.id = plane_id;
    plane.type = visualization_msgs::Marker::SPHERE;
    plane.action = visualization_msgs::Marker::ADD;
    plane.pose.position.x = octo->plane_center[0];
    plane.pose.position.y = octo->plane_center[1];
    plane.pose.position.z = octo->plane_center[2];      
    auto q = Eigen::Quaterniond::FromTwoVectors(V3D(1,0,0),saes.eigenvectors().col(0));
    plane.pose.orientation.w = q.w();
    plane.pose.orientation.x = q.x();
    plane.pose.orientation.y = q.y();
    plane.pose.orientation.z = q.z();
    plane.scale.x = 2*sqrt(saes.eigenvalues()[0]);
    plane.scale.y = 3*sqrt(saes.eigenvalues()[1]);
    plane.scale.z = 3*sqrt(saes.eigenvalues()[2]);
    plane.color.a = 1.0;
    plane.color.r = plane_rgb(0);
    plane.color.g = plane_rgb(1);
    plane.color.b = plane_rgb(2);
    plane.lifetime = ros::Duration();
    voxel_plane.markers.push_back(plane);
    plane_id++;
  }
  for (int i=0;i<8;i++) {
    if(octo->leaves[i] != nullptr) saveVoxel(octo->leaves[i], voxel_plane, plane_id);
  }
}

void pubVoxelMap(const std::unordered_map<VOXEL_LOC, OCTO_TREE*> &voxel_map,
                const int pub_max_voxel_layer, const ros::Publisher &plane_map_pub) {
  visualization_msgs::MarkerArray voxel_plane;
  uint plane_id = 0;
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {
    saveVoxel(iter->second, voxel_plane, plane_id);
  }
  plane_map_pub.publish(voxel_plane);
}
#endif