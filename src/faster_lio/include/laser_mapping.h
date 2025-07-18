#ifndef FASTER_LIO_LASER_MAPPING_H
#define FASTER_LIO_LASER_MAPPING_H

#include <condition_variable>
#include <thread>

#include <nav_msgs/msg/path.hpp>
#include <pcl/filters/voxel_grid.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "faster_lio_interfaces/msg/custom_msg.hpp"
#include "imu_processing.hpp"
#include "ivox3d/ivox3d.h"
#include "options.h"
#include "pointcloud_preprocess.h"

// namespace livox_ros_driver {
// // Minimal stub for CustomMsg to allow build without livox_ros_driver
// struct CustomMsg {
//     using SharedPtr = std::shared_ptr<CustomMsg>;
//     using ConstSharedPtr = std::shared_ptr<const CustomMsg>;
//     struct Header {
//         struct Stamp {
//             double stamp;
//             double seconds() const { return stamp; }
//             Stamp() : stamp(0) {}
//         } stamp;
//         Header() = default;
//     } header;
//     CustomMsg() = default;
//     // Add dummy fields as needed for your code to compile
// };
// }

namespace faster_lio {

class LaserMapping : public rclcpp::Node {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

#ifdef IVOX_NODE_TYPE_PHC
    using IVoxType = IVox<3, IVoxNodeType::PHC, PointType>;
#else
    using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;
#endif

    LaserMapping();
    ~LaserMapping();

    /// init with ros2
    bool InitROS2();

    /// init without ros
    bool InitWithoutROS(const std::string &config_yaml);

    void Run();

    // callbacks of lidar and imu
    void StandardPCLCallBack(sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void LivoxPCLCallBack(faster_lio_interfaces::msg::CustomMsg::SharedPtr msg);
    void IMUCallBack(sensor_msgs::msg::Imu::SharedPtr msg_in);
    void Mid360IMUCallBack(sensor_msgs::msg::Imu::SharedPtr msg_in);

    // sync lidar with imu
    bool SyncPackages();

    /// interface of mtk, customized obseravtion model
    void ObsModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);

    ////////////////////////////// debug save / show ////////////////////////////////////////////////////////////////
    void PublishState(const state_ikfom& state) const;
    void PublishPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path);
    void PublishOdometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_path);
    void PublishFrameWorld();
    void PublishFrameBody(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_path);
    void PublishFrameEffectWorld(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_path);
    void Savetrajectory(const std::string &traj_file);

    void Finish();

   private:
    template <typename T>
    void SetPosestamp(T &out);

    void PointBodyToWorld(PointType const *pi, PointType *const po);
    void PointBodyToWorld(const common::V3F &pi, PointType *const po);
    void PointBodyLidarToIMU(PointType const *const pi, PointType *const po);

    void MapIncremental();

    void SubAndPubToROS2();

    bool LoadParams();
    bool LoadParamsFromYAML(const std::string &yaml);

    void PrintState(const state_ikfom &s);

   private:
    /// modules
    IVoxType::Options ivox_options_;
    std::shared_ptr<IVoxType> ivox_ = nullptr;                    // localmap in ivox
    std::shared_ptr<PointCloudPreprocess> preprocess_ = nullptr;  // point cloud preprocess
    std::shared_ptr<ImuProcess> p_imu_ = nullptr;                 // imu process

    /// local map related
    float det_range_ = 300.0f;
    double cube_len_ = 0;
    double filter_size_map_min_ = 0;
    bool localmap_initialized_ = false;

    /// params
    std::string lidar_topic_;
    std::string imu_topic_;
    std::vector<double> extrinT_{3, 0.0};  // lidar-imu translation
    std::vector<double> extrinR_{9, 0.0};  // lidar-imu rotation

    /// point clouds data
    CloudPtr scan_undistort_{new PointCloudType()};   // scan after undistortion
    CloudPtr scan_down_body_{new PointCloudType()};   // downsampled scan in body
    CloudPtr scan_down_world_{new PointCloudType()};  // downsampled scan in world
    std::vector<PointVector> nearest_points_;         // nearest points of current scan
    common::VV4F corr_pts_;                           // inlier pts
    common::VV4F corr_norm_;                          // inlier plane norms
    pcl::VoxelGrid<PointType> voxel_scan_;            // voxel filter for current scan
    std::vector<float> residuals_;                    // point-to-plane residuals
    std::vector<bool> point_selected_surf_;           // selected points
    common::VV4F plane_coef_;                         // plane coeffs

    /// ros2 pub and sub stuffs
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_;
    rclcpp::Subscription<faster_lio_interfaces::msg::CustomMsg>::SharedPtr sub_livox_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_world_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_effect_world_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_aft_mapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_state_velocity_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_state_acc_bias_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_state_gyr_bias_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string tf_imu_frame_;
    std::string tf_world_frame_;

    std::mutex mtx_buffer_;
    std::deque<double> time_buffer_;
    std::deque<PointCloudType::Ptr> lidar_buffer_;
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_buffer_;
    nav_msgs::msg::Odometry odom_aft_mapped_;

    /// options
    bool time_sync_en_ = false;
    double timediff_lidar_wrt_imu_ = 0.0;
    double last_timestamp_lidar_ = 0;
    double lidar_end_time_ = 0;
    double last_timestamp_imu_ = -1.0;
    double first_lidar_time_ = 0.0;
    bool lidar_pushed_ = false;

    /// statistics and flags ///
    int scan_count_ = 0;
    int publish_count_ = 0;
    bool flg_first_scan_ = true;
    bool flg_EKF_inited_ = false;
    int pcd_index_ = 0;
    double lidar_mean_scantime_ = 0.0;
    int scan_num_ = 0;
    bool timediff_set_flg_ = false;
    int effect_feat_num_ = 0, frame_num_ = 0;

    ///////////////////////// EKF inputs and output ///////////////////////////////////////////////////////
    common::MeasureGroup measures_;                    // sync IMU and lidar scan
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf_;  // esekf
    state_ikfom state_point_;                          // ekf current state
    vect3 pos_lidar_;                                  // lidar position after eskf update
    common::V3D euler_cur_ = common::V3D::Zero();      // rotation in euler angles
    bool extrinsic_est_en_ = true;

    /////////////////////////  debug show / save /////////////////////////////////////////////////////////
    bool run_in_offline_ = false;
    bool path_pub_en_ = true;
    bool scan_pub_en_ = false;
    bool dense_pub_en_ = false;
    bool scan_body_pub_en_ = false;
    bool scan_effect_pub_en_ = false;
    bool pcd_save_en_ = false;
    bool runtime_pos_log_ = true;
    int pcd_save_interval_ = -1;
    bool path_save_en_ = false;
    std::string dataset_;

    PointCloudType::Ptr pcl_wait_save_{new PointCloudType()};  // debug save
    nav_msgs::msg::Path path_;
    geometry_msgs::msg::PoseStamped msg_body_pose_;
};

}  // namespace faster_lio

#endif  // FASTER_LIO_LASER_MAPPING_H