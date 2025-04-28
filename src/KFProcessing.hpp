#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.hpp>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include "use-ikfom.hpp"

const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};
const bool norm_list(std::pair<int,double> &x, std::pair<int,double> &y) {return (x.second > y.second);};

int lidar_num = 1;
std::vector<Eigen::Vector3d> Textrinsic;
std::vector<Eigen::Matrix<double,3,3>> Rextrinsic;


class KFProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  KFProcess();
  ~KFProcess();
  
  void Reset();
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4,4) &T);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  Eigen::Matrix<double, 6, 6> Q;
  void Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 6> &kf_state, 
              PointCloudXYZI::Ptr pcl_un_, bool flg_EKF_inited);

  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  M3D Lidar_R_wrt_IMU;
  V3D Lidar_T_wrt_IMU;
  double first_lidar_time;
 private:
  void init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 6> &kf_state);
  void pointLidarToLidar(PointType const * const pi, PointType * const po, const int lidar_id);
  double last_lidar_beg_time_ = -1.0;
  bool kf_need_init_ = true;
};

KFProcess::KFProcess(): kf_need_init_(true)
{
  Q = process_noise_cov();  
  cov_acc       = V3D(0.1, 0.1, 0.1);
  cov_gyr       = V3D(0.1, 0.1, 0.1);
  Lidar_T_wrt_IMU = Zero3d;
  Lidar_R_wrt_IMU = Eye3d;
}

KFProcess::~KFProcess() {}

void KFProcess::Reset() 
{
  kf_need_init_ = true;
  Q.block<3, 3>(0, 0).diagonal() = cov_gyr_scale;
  Q.block<3, 3>(3, 3).diagonal() = cov_acc_scale;
}

void KFProcess::set_extrinsic(const MD(4,4) &T)
{
  Lidar_T_wrt_IMU = T.block<3,1>(0,3);
  Lidar_R_wrt_IMU = T.block<3,3>(0,0);
}

void KFProcess::set_extrinsic(const V3D &transl)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU.setIdentity();
}

void KFProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
}

void KFProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyr_scale = scaler;
}

void KFProcess::set_acc_cov(const V3D &scaler)
{
  cov_acc_scale = scaler;
}

void KFProcess::init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 6> &kf_state) {
  state_ikfom init_state = kf_state.get_x();
//   init_state.rot = Eye3d;
//   init_state.pos = Zero3d;
  //! jin: initial pose
  init_state.rot = Eigen::Quaterniond(0.92388, -7.96122e-09, -1.82915e-09, 0.382683).normalized().toRotationMatrix();
  init_state.pos = V3D(0.0, 0.0, 0.75);

  init_state.vel = Zero3d;
  init_state.omg = Zero3d;
  init_state.acc = Zero3d;
  kf_state.change_x(init_state);

  esekfom::esekf<state_ikfom, 6>::cov init_P = kf_state.get_P();
  init_P.setIdentity();
  init_P(0,0) = init_P(1,1) = init_P(2,2) = 0.01;       //pos
  init_P(3,3) = init_P(4,4) = init_P(5,5) = 0.001;      //rot
  init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.01;       //vel
  init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.01;   //omg
  init_P(12,12) = init_P(13,13) = init_P(14,14) = 0.01; //acc
  kf_state.change_P(init_P);
}

void KFProcess::pointLidarToLidar(PointType const * const pi, PointType * const po, const int lidar_id) {
  *po = *pi;
  po->id = lidar_id;
  if (lidar_id) {
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global = Rextrinsic[lidar_id]*p_body + Textrinsic[lidar_id];
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
  }
}

void KFProcess::Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 6> &kf_state, 
                        PointCloudXYZI::Ptr cur_pcl_un_, bool flg_EKF_inited) {
  if (kf_need_init_) {
    init(meas, kf_state);
    kf_need_init_ = false;
    last_lidar_beg_time_ = meas.lidar_beg_time;
    return;
  }

  const double &pcl_beg_time = meas.lidar_beg_time;
  /*** sort point clouds by offset time ***/
  cur_pcl_un_->clear();
  for (int i=0; i<lidar_num; i++) {
    PointCloudXYZI feats_lidar;
    feats_lidar.resize(meas.lidar[i].size());
    for (int j=0; j<meas.lidar[i].size(); j++) {
      pointLidarToLidar(&(meas.lidar[i].points[j]), &(feats_lidar.points[j]), i);
    }
    *cur_pcl_un_ += feats_lidar;
  }

  double dt;  
  dt = last_lidar_beg_time_>=0 ? (pcl_beg_time - last_lidar_beg_time_) : 0.0;
  kf_state.predict(dt, Q);
  last_lidar_beg_time_ = pcl_beg_time;
}