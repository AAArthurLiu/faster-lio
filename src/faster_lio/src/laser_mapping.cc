#include <tf2_ros/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>
#include <execution>
#include <fstream>

#include "laser_mapping.h"
#include "common_lib.h"
#include "utils.h"

namespace faster_lio {

namespace {
constexpr double kGravity = 9.80655;  // m/s^2
}  // namespace

LaserMapping::LaserMapping() : rclcpp::Node("laser_mapping") {
    preprocess_ = std::make_shared<PointCloudPreprocess>();
    p_imu_ = std::make_shared<ImuProcess>();
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

LaserMapping::~LaserMapping() {
    scan_down_body_ = nullptr;
    scan_undistort_ = nullptr;
    scan_down_world_ = nullptr;
    RCLCPP_INFO(this->get_logger(), "laser mapping deconstruct");
}

bool LaserMapping::InitROS2() {
    LoadParams();
    SubAndPubToROS2();

    // localmap init (after LoadParams)
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    // esekf init
    std::vector<double> epsi(23, 0.001);
    kf_.init_dyn_share(
        get_f, df_dx, df_dw,
        [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) { ObsModel(s, ekfom_data); },
        options::NUM_MAX_ITERATIONS, epsi.data());

    return true;
}

bool LaserMapping::InitWithoutROS(const std::string &config_yaml) {
    LOG(INFO) << "init laser mapping from " << config_yaml;
    if (!LoadParamsFromYAML(config_yaml)) {
        return false;
    }

    // localmap init (after LoadParams)
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    // esekf init
    std::vector<double> epsi(23, 0.001);
    kf_.init_dyn_share(
        get_f, df_dx, df_dw,
        [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) { ObsModel(s, ekfom_data); },
        options::NUM_MAX_ITERATIONS, epsi.data());

    if (std::is_same<IVoxType, IVox<3, IVoxNodeType::PHC, pcl::PointXYZI>>::value == true) {
        LOG(INFO) << "using phc ivox";
    } else if (std::is_same<IVoxType, IVox<3, IVoxNodeType::DEFAULT, pcl::PointXYZI>>::value == true) {
        LOG(INFO) << "using default ivox";
    }

    return true;
}

bool LaserMapping::LoadParams() {
    // Use ROS2 parameter API
    this->declare_parameter("path_save_en", true);
    this->declare_parameter("publish.path_publish_en", true);
    this->declare_parameter("publish.scan_publish_en", true);
    this->declare_parameter("publish.dense_publish_en", false);
    this->declare_parameter("publish.scan_bodyframe_pub_en", true);
    this->declare_parameter("publish.scan_effect_pub_en", false);
    this->declare_parameter("publish.tf_imu_frame", std::string("body"));
    this->declare_parameter("publish.tf_world_frame", std::string("camera_init"));
    this->declare_parameter("max_iteration", 4);
    this->declare_parameter("esti_plane_threshold", 0.1f);
    this->declare_parameter("common.lidar_topic", "");
    this->declare_parameter("common.imu_topic", "");
    this->declare_parameter("common.time_sync_en", false);
    this->declare_parameter("filter_size_surf", 0.5);
    this->declare_parameter("filter_size_map", 0.0);
    this->declare_parameter("cube_side_length", 200.0);
    this->declare_parameter("mapping.det_range", 300.0f);
    this->declare_parameter("mapping.gyr_cov", 0.1);
    this->declare_parameter("mapping.acc_cov", 0.1);
    this->declare_parameter("mapping.b_gyr_cov", 0.0001);
    this->declare_parameter("mapping.b_acc_cov", 0.0001);
    this->declare_parameter("preprocess.blind", 0.01);
    this->declare_parameter("preprocess.time_scale", 1e-3f);
    this->declare_parameter("preprocess.lidar_type", 1);
    this->declare_parameter("preprocess.scan_line", 16);
    this->declare_parameter("point_filter_num", 2);
    this->declare_parameter("feature_extract_enable", false);
    this->declare_parameter("runtime_pos_log_enable", true);
    this->declare_parameter("mapping.extrinsic_est_en", true);
    this->declare_parameter("pcd_save.pcd_save_en", false);
    this->declare_parameter("pcd_save.interval", -1);
    this->declare_parameter("mapping.extrinsic_T", std::vector<double>());
    this->declare_parameter("mapping.extrinsic_R", std::vector<double>());
    this->declare_parameter("ivox_grid_resolution", 0.2f);
    this->declare_parameter("ivox_nearby_type", 18);

    // Get parameters
    this->get_parameter("path_save_en", path_save_en_);
    this->get_parameter("publish.path_publish_en", path_pub_en_);
    this->get_parameter("publish.scan_publish_en", scan_pub_en_);
    this->get_parameter("publish.scan_effect_pub_en", scan_effect_pub_en_);
    this->get_parameter("publish.dense_publish_en", dense_pub_en_);
    this->get_parameter("publish.scan_bodyframe_pub_en", scan_body_pub_en_);
    this->get_parameter("publish.tf_imu_frame", tf_imu_frame_);
    this->get_parameter("publish.tf_world_frame", tf_world_frame_);
    this->get_parameter("max_iteration", options::NUM_MAX_ITERATIONS);
    this->get_parameter("esti_plane_threshold", options::ESTI_PLANE_THRESHOLD);
    this->get_parameter("common.lidar_topic", lidar_topic_);
    this->get_parameter("common.imu_topic", imu_topic_);
    this->get_parameter("common.time_sync_en", time_sync_en_);
    double filter_size_surf_min;
    this->get_parameter("filter_size_surf", filter_size_surf_min);
    this->get_parameter("filter_size_map", filter_size_map_min_);
    this->get_parameter("cube_side_length", cube_len_);
    this->get_parameter("mapping.det_range", det_range_);
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    this->get_parameter("mapping.gyr_cov", gyr_cov);
    this->get_parameter("mapping.acc_cov", acc_cov);
    this->get_parameter("mapping.b_gyr_cov", b_gyr_cov);
    this->get_parameter("mapping.b_acc_cov", b_acc_cov);
    this->get_parameter("preprocess.blind", preprocess_->Blind());
    double time_scale = 1;
    this->get_parameter("preprocess.time_scale", time_scale);
    int lidar_type, scan_line, point_filter_num;
    this->get_parameter("preprocess.lidar_type", lidar_type);
    this->get_parameter("preprocess.scan_line", scan_line);
    this->get_parameter("point_filter_num", point_filter_num);
    bool feature_extract_enable;
    this->get_parameter("feature_extract_enable", feature_extract_enable);
    this->get_parameter("runtime_pos_log_enable", runtime_pos_log_);
    this->get_parameter("mapping.extrinsic_est_en", extrinsic_est_en_);
    this->get_parameter("pcd_save.pcd_save_en", pcd_save_en_);
    this->get_parameter("pcd_save.interval", pcd_save_interval_);
    this->get_parameter("mapping.extrinsic_T", extrinT_);
    this->get_parameter("mapping.extrinsic_R", extrinR_);
    this->get_parameter("ivox_grid_resolution", ivox_options_.resolution_);
    int ivox_nearby_type;
    this->get_parameter("ivox_nearby_type", ivox_nearby_type);

    preprocess_->SetLidarType(lidar_type);

    if (ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 18) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    } else if (ivox_nearby_type == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    path_.header.stamp = this->now();
    CHECK(!tf_world_frame_.empty());
    path_.header.frame_id = tf_world_frame_;

    voxel_scan_.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

    CHECK(!extrinT_.empty());
    CHECK(!extrinR_.empty());
    common::V3D lidar_T_wrt_IMU = common::VecFromArray<double>(extrinT_);
    common::M3D lidar_R_wrt_IMU = common::MatFromArray<double>(extrinR_);

    p_imu_->SetExtrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);
    p_imu_->SetGyrCov(common::V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetAccCov(common::V3D(acc_cov, acc_cov, acc_cov));
    p_imu_->SetGyrBiasCov(common::V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccBiasCov(common::V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    return true;
}

bool LaserMapping::LoadParamsFromYAML(const std::string &yaml_file) {
    LOG(FATAL) << "To implement lidar_type new enums and lidar, imu topic assignment";
    // get params from yaml
    int lidar_type, ivox_nearby_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_surf_min;
    common::V3D lidar_T_wrt_IMU;
    common::M3D lidar_R_wrt_IMU;

    auto yaml = YAML::LoadFile(yaml_file);
    try {
        path_pub_en_ = yaml["publish"]["path_publish_en"].as<bool>();
        scan_pub_en_ = yaml["publish"]["scan_publish_en"].as<bool>();
        dense_pub_en_ = yaml["publish"]["dense_publish_en"].as<bool>();
        scan_body_pub_en_ = yaml["publish"]["scan_bodyframe_pub_en"].as<bool>();
        scan_effect_pub_en_ = yaml["publish"]["scan_effect_pub_en"].as<bool>();
        tf_imu_frame_ = yaml["publish"]["tf_imu_frame"].as<std::string>("body");
        tf_world_frame_ = yaml["publish"]["tf_world_frame"].as<std::string>("camera_init");
        path_save_en_ = yaml["path_save_en"].as<bool>();

        options::NUM_MAX_ITERATIONS = yaml["max_iteration"].as<int>();
        options::ESTI_PLANE_THRESHOLD = yaml["esti_plane_threshold"].as<float>();
        time_sync_en_ = yaml["common"]["time_sync_en"].as<bool>();

        filter_size_surf_min = yaml["filter_size_surf"].as<float>();
        filter_size_map_min_ = yaml["filter_size_map"].as<float>();
        cube_len_ = yaml["cube_side_length"].as<int>();
        det_range_ = yaml["mapping"]["det_range"].as<float>();
        gyr_cov = yaml["mapping"]["gyr_cov"].as<float>();
        acc_cov = yaml["mapping"]["acc_cov"].as<float>();
        b_gyr_cov = yaml["mapping"]["b_gyr_cov"].as<float>();
        b_acc_cov = yaml["mapping"]["b_acc_cov"].as<float>();
        preprocess_->Blind() = yaml["preprocess"]["blind"].as<double>();
        preprocess_->TimeScale() = yaml["preprocess"]["time_scale"].as<double>();
        lidar_type = yaml["preprocess"]["lidar_type"].as<int>();
        preprocess_->NumScans() = yaml["preprocess"]["scan_line"].as<int>();
        preprocess_->PointFilterNum() = yaml["point_filter_num"].as<int>();
        preprocess_->FeatureEnabled() = yaml["feature_extract_enable"].as<bool>();
        extrinsic_est_en_ = yaml["mapping"]["extrinsic_est_en"].as<bool>();
        pcd_save_en_ = yaml["pcd_save"]["pcd_save_en"].as<bool>();
        pcd_save_interval_ = yaml["pcd_save"]["interval"].as<int>();
        extrinT_ = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
        extrinR_ = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();

        ivox_options_.resolution_ = yaml["ivox_grid_resolution"].as<float>();
        ivox_nearby_type = yaml["ivox_nearby_type"].as<int>();
    } catch (...) {
        LOG(ERROR) << "bad conversion";
        return false;
    }

    preprocess_->SetLidarType(lidar_type);

    if (ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 18) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    } else if (ivox_nearby_type == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    voxel_scan_.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

    lidar_T_wrt_IMU = common::VecFromArray<double>(extrinT_);
    lidar_R_wrt_IMU = common::MatFromArray<double>(extrinR_);

    p_imu_->SetExtrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);
    p_imu_->SetGyrCov(common::V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetAccCov(common::V3D(acc_cov, acc_cov, acc_cov));
    p_imu_->SetGyrBiasCov(common::V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccBiasCov(common::V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    run_in_offline_ = true;
    return true;
}

void LaserMapping::SubAndPubToROS2() {
    // ROS2 subscribe initialization
    CHECK(!lidar_topic_.empty());
    CHECK(!imu_topic_.empty());
    LOG(INFO) << "Subscribing to lidar topic: " << lidar_topic_;
    LOG(INFO) << "Subscribing to imu topic: " << imu_topic_;

    if (preprocess_->GetLidarType() == LidarType::AVIA) {
        sub_livox_ = this->create_subscription<faster_lio_interfaces::msg::CustomMsg>(
            lidar_topic_, rclcpp::SensorDataQoS(),
            std::bind(&LaserMapping::LivoxPCLCallBack, this, std::placeholders::_1));
    } else {
        sub_pcl_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_, rclcpp::SensorDataQoS(),
            std::bind(&LaserMapping::StandardPCLCallBack, this, std::placeholders::_1));
    }

    if (preprocess_->GetLidarType() == LidarType::MID360) {
        LOG(INFO) << "Using MID360 Lidar's IMU";
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, rclcpp::SensorDataQoS(),
            std::bind(&LaserMapping::Mid360IMUCallBack, this, std::placeholders::_1));
    } else {
        LOG(INFO) << "Using regular IMU";
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, rclcpp::SensorDataQoS(),
            std::bind(&LaserMapping::IMUCallBack, this, std::placeholders::_1));
    }

    // ROS2 publisher init
    pub_laser_cloud_world_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 10);
    pub_laser_cloud_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 10);
    pub_laser_cloud_effect_world_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_effect_world", 10);
    pub_odom_aft_mapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 10);
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
    pub_state_velocity_ = this->create_publisher<geometry_msgs::msg::Vector3>("/state_velocity", 10);
    pub_state_acc_bias_ = this->create_publisher<geometry_msgs::msg::Vector3>("/state_acc_bias", 10);
    pub_state_gyr_bias_ = this->create_publisher<geometry_msgs::msg::Vector3>("/state_gyr_bias", 10);
}

void LaserMapping::Run() {
    if (!SyncPackages()) {
        return;
    }

    /// IMU process, kf prediction, undistortion
    p_imu_->Process(measures_, kf_, scan_undistort_);
    if (scan_undistort_->empty() || (scan_undistort_ == nullptr)) {
        LOG(WARNING) << "No point, skip this scan!";
        return;
    }

    /// the first scan
    if (flg_first_scan_) {
        state_point_ = kf_.get_x();
        scan_down_world_->resize(scan_undistort_->size());
        for (int i = 0; i < scan_undistort_->size(); i++) {
            PointBodyToWorld(&scan_undistort_->points[i], &scan_down_world_->points[i]);
        }
        ivox_->AddPoints(scan_down_world_->points);
        first_lidar_time_ = measures_.lidar_bag_time_;
        flg_first_scan_ = false;
        return;
    }
    flg_EKF_inited_ = (measures_.lidar_bag_time_ - first_lidar_time_) >= options::INIT_TIME;

    /// downsample
    Timer::Evaluate(
        [&, this]() {
            voxel_scan_.setInputCloud(scan_undistort_);
            voxel_scan_.filter(*scan_down_body_);
        },
        "Downsample PointCloud");

    int cur_pts = scan_down_body_->size();
    if (cur_pts < 5) {
        LOG(WARNING) << "Too few points, skip this scan!" << scan_undistort_->size() << ", " << scan_down_body_->size();
        return;
    }
    scan_down_world_->resize(cur_pts);
    nearest_points_.resize(cur_pts);
    residuals_.resize(cur_pts, 0);
    point_selected_surf_.resize(cur_pts, true);
    plane_coef_.resize(cur_pts, common::V4F::Zero());

    // ICP and iterated Kalman filter update
    Timer::Evaluate(
        [&, this]() {
            // iterated state estimation
            double solve_H_time = 0;
            // update the observation model, will call nn and point-to-plane residual computation
            kf_.update_iterated_dyn_share_modified(options::LASER_POINT_COV, solve_H_time);
            // save the state
            state_point_ = kf_.get_x();
            euler_cur_ = SO3ToEuler(state_point_.rot);
            pos_lidar_ = state_point_.pos + state_point_.rot * state_point_.offset_T_L_I;
        },
        "IEKF Solve and Update");

    // update local map
    Timer::Evaluate([&, this]() { MapIncremental(); }, "    Incremental Mapping");

    LOG_EVERY_N(INFO, 100) << "[ mapping ]: In num: " << scan_undistort_->points.size() << " downsamp " << cur_pts
                           << " Map grid num: " << ivox_->NumValidGrids() << " effect num : " << effect_feat_num_;

    // publish or save map pcd
    if (run_in_offline_) {
        if (pcd_save_en_) {
            PublishFrameWorld();
        }
        if (path_save_en_) {
            PublishPath(pub_path_);
        }
    } else {
        PublishState(state_point_);
        if (pub_odom_aft_mapped_) {
            PublishOdometry(pub_odom_aft_mapped_);
        }
        if (path_pub_en_ || path_save_en_) {
            PublishPath(pub_path_);
        }
        if (scan_pub_en_ || pcd_save_en_) {
            PublishFrameWorld();
        }
        if (scan_pub_en_ && scan_body_pub_en_) {
            PublishFrameBody(pub_laser_cloud_body_);
        }
        if (scan_pub_en_ && scan_effect_pub_en_) {
            PublishFrameEffectWorld(pub_laser_cloud_effect_world_);
        }
    }

    // Debug variables
    frame_num_++;
}

void LaserMapping::StandardPCLCallBack( sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_buffer_);
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;
            if (common::GetSecondsFromHeader(msg->header) < last_timestamp_lidar_) {
                LOG(ERROR) << "lidar loop back, skip current lidar";
                return;
            }

            PointCloudType::Ptr ptr(new PointCloudType());
            preprocess_->Process(msg, ptr);
            lidar_buffer_.push_back(ptr);
            time_buffer_.push_back(common::GetSecondsFromHeader(msg->header));
            last_timestamp_lidar_ = common::GetSecondsFromHeader(msg->header);
        },
        "Preprocess (Standard)");
}

void LaserMapping::LivoxPCLCallBack( faster_lio_interfaces::msg::CustomMsg::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_buffer_);
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;
            if (common::GetSecondsFromHeader(msg->header) < last_timestamp_lidar_) {
                LOG(ERROR) << "lidar loop back, skip current lidar";
                return;
            }

            last_timestamp_lidar_ = common::GetSecondsFromHeader(msg->header);

            if (!time_sync_en_ && abs(last_timestamp_imu_ - last_timestamp_lidar_) > 10.0 && !imu_buffer_.empty() &&
                !lidar_buffer_.empty()) {
                LOG(INFO) << "IMU and LiDAR not Synced, IMU time: " << last_timestamp_imu_
                          << ", lidar header time: " << last_timestamp_lidar_;
            }

            if (time_sync_en_ && !timediff_set_flg_ && abs(last_timestamp_lidar_ - last_timestamp_imu_) > 1 &&
                !imu_buffer_.empty()) {
                timediff_set_flg_ = true;
                timediff_lidar_wrt_imu_ = last_timestamp_lidar_ + 0.1 - last_timestamp_imu_;
                LOG(INFO) << "Self sync IMU and LiDAR, time diff is " << timediff_lidar_wrt_imu_;
            }

            PointCloudType::Ptr ptr(new PointCloudType());
            preprocess_->Process(msg, ptr);
            lidar_buffer_.emplace_back(ptr);
            time_buffer_.emplace_back(last_timestamp_lidar_);
        },
        "Preprocess (Livox)");

}

void LaserMapping::IMUCallBack( sensor_msgs::msg::Imu::SharedPtr msg_in) {
    sensor_msgs::msg::Imu::SharedPtr msg = std::make_shared<sensor_msgs::msg::Imu>(*msg_in);

    if (abs(timediff_lidar_wrt_imu_) > 0.1 && time_sync_en_) {
        msg->header.stamp = common::SecondsToTimestamp(timediff_lidar_wrt_imu_ + common::GetSecondsFromHeader(msg_in->header));
    }

    double timestamp = common::GetSecondsFromHeader(msg->header);

    std::lock_guard<std::mutex> lock(mtx_buffer_);
    if (timestamp < last_timestamp_imu_) {
        LOG(ERROR) << "imu loop back, skip current imu";
        return;
    }

    publish_count_++;

    last_timestamp_imu_ = timestamp;
    imu_buffer_.emplace_back(msg);
}

void LaserMapping::Mid360IMUCallBack(sensor_msgs::msg::Imu::SharedPtr msg_in) {
    sensor_msgs::msg::Imu::SharedPtr msg = std::make_shared<sensor_msgs::msg::Imu>(*msg_in);
    // Multiple the acc part of IMU by gravity to convert to m/s^2
    msg->linear_acceleration.x *= kGravity;
    msg->linear_acceleration.y *= kGravity;
    msg->linear_acceleration.z *= kGravity;
    IMUCallBack(msg);
}

bool LaserMapping::SyncPackages() {
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed_) {
        measures_.lidar_ = lidar_buffer_.front();
        measures_.lidar_bag_time_ = time_buffer_.front();

        if (measures_.lidar_->points.size() <= 1) {
            LOG(WARNING) << "Too few input point cloud!";
            lidar_end_time_ = measures_.lidar_bag_time_ + lidar_mean_scantime_;
        } else if (measures_.lidar_->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime_) {
            lidar_end_time_ = measures_.lidar_bag_time_ + lidar_mean_scantime_;
        } else {
            scan_num_++;
            lidar_end_time_ = measures_.lidar_bag_time_ + measures_.lidar_->points.back().curvature / double(1000);
            lidar_mean_scantime_ +=
                (measures_.lidar_->points.back().curvature / double(1000) - lidar_mean_scantime_) / scan_num_;
        }

        measures_.lidar_end_time_ = lidar_end_time_;
        lidar_pushed_ = true;
    }

    if (last_timestamp_imu_ < lidar_end_time_) {
        return false;
    }

    /*** push imu_ data, and pop from imu_ buffer ***/
    double imu_time = common::GetSecondsFromHeader(imu_buffer_.front()->header);
    measures_.imu_.clear();
    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) {
        imu_time = common::GetSecondsFromHeader(imu_buffer_.front()->header);
        if (imu_time > lidar_end_time_) break;
        measures_.imu_.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
    }

    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    lidar_pushed_ = false;
    return true;
}

void LaserMapping::PrintState(const state_ikfom &s) {
    LOG(INFO) << "state r: " << s.rot.coeffs().transpose() << ", t: " << s.pos.transpose()
              << ", off r: " << s.offset_R_L_I.coeffs().transpose() << ", t: " << s.offset_T_L_I.transpose();
}

void LaserMapping::MapIncremental() {
    PointVector points_to_add;
    PointVector point_no_need_downsample;

    int cur_pts = scan_down_body_->size();
    points_to_add.reserve(cur_pts);
    point_no_need_downsample.reserve(cur_pts);

    std::vector<size_t> index(cur_pts);
    for (size_t i = 0; i < cur_pts; ++i) {
        index[i] = i;
    }

    std::for_each(std::execution::unseq, index.begin(), index.end(), [&](const size_t &i) {
        /* transform to world frame */
        PointBodyToWorld(&(scan_down_body_->points[i]), &(scan_down_world_->points[i]));

        /* decide if need add to map */
        PointType &point_world = scan_down_world_->points[i];
        if (!nearest_points_[i].empty() && flg_EKF_inited_) {
            const PointVector &points_near = nearest_points_[i];

            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / filter_size_map_min_).array().floor() + 0.5) * filter_size_map_min_;

            Eigen::Vector3f dis_2_center = points_near[0].getVector3fMap() - center;

            if (fabs(dis_2_center.x()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.y()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.z()) > 0.5 * filter_size_map_min_) {
                point_no_need_downsample.emplace_back(point_world);
                return;
            }

            bool need_add = true;
            float dist = common::calc_dist(point_world.getVector3fMap(), center);
            if (points_near.size() >= options::NUM_MATCH_POINTS) {
                for (int readd_i = 0; readd_i < options::NUM_MATCH_POINTS; readd_i++) {
                    if (common::calc_dist(points_near[readd_i].getVector3fMap(), center) < dist + 1e-6) {
                        need_add = false;
                        break;
                    }
                }
            }
            if (need_add) {
                points_to_add.emplace_back(point_world);
            }
        } else {
            points_to_add.emplace_back(point_world);
        }
    });

    Timer::Evaluate(
        [&, this]() {
            ivox_->AddPoints(points_to_add);
            ivox_->AddPoints(point_no_need_downsample);
        },
        "    IVox Add Points");
}

/**
 * Lidar point cloud registration
 * will be called by the eskf custom observation model
 * compute point-to-plane residual here
 * @param s kf state
 * @param ekfom_data H matrix
 */
void LaserMapping::ObsModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
    int cnt_pts = scan_down_body_->size();

    std::vector<size_t> index(cnt_pts);
    for (size_t i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    Timer::Evaluate(
        [&, this]() {
            auto R_wl = (s.rot * s.offset_R_L_I).cast<float>();
            auto t_wl = (s.rot * s.offset_T_L_I + s.pos).cast<float>();

            /** closest surface search and residual computation **/
            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                PointType &point_body = scan_down_body_->points[i];
                PointType &point_world = scan_down_world_->points[i];

                /* transform to world frame */
                common::V3F p_body = point_body.getVector3fMap();
                point_world.getVector3fMap() = R_wl * p_body + t_wl;
                point_world.intensity = point_body.intensity;

                auto &points_near = nearest_points_[i];
                if (ekfom_data.converge) {
                    /** Find the closest surfaces in the map **/
                    ivox_->GetClosestPoint(point_world, points_near, options::NUM_MATCH_POINTS);
                    point_selected_surf_[i] = points_near.size() >= options::MIN_NUM_MATCH_POINTS;
                    if (point_selected_surf_[i]) {
                        point_selected_surf_[i] =
                            common::esti_plane(plane_coef_[i], points_near, options::ESTI_PLANE_THRESHOLD);
                    }
                }

                if (point_selected_surf_[i]) {
                    auto temp = point_world.getVector4fMap();
                    temp[3] = 1.0;
                    float pd2 = plane_coef_[i].dot(temp);

                    bool valid_corr = p_body.norm() > 81 * pd2 * pd2;
                    if (valid_corr) {
                        point_selected_surf_[i] = true;
                        residuals_[i] = pd2;
                    }
                }
            });
        },
        "    ObsModel (Lidar Match)");

    effect_feat_num_ = 0;

    corr_pts_.resize(cnt_pts);
    corr_norm_.resize(cnt_pts);
    for (int i = 0; i < cnt_pts; i++) {
        if (point_selected_surf_[i]) {
            corr_norm_[effect_feat_num_] = plane_coef_[i];
            corr_pts_[effect_feat_num_] = scan_down_body_->points[i].getVector4fMap();
            corr_pts_[effect_feat_num_][3] = residuals_[i];

            effect_feat_num_++;
        }
    }
    corr_pts_.resize(effect_feat_num_);
    corr_norm_.resize(effect_feat_num_);

    if (effect_feat_num_ < 1) {
        ekfom_data.valid = false;
        LOG(WARNING) << "No Effective Points!";
        return;
    }

    Timer::Evaluate(
        [&, this]() {
            /*** Computation of Measurement Jacobian matrix H and measurements vector ***/
            ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_feat_num_, 12);  // 23
            ekfom_data.h.resize(effect_feat_num_);

            index.resize(effect_feat_num_);
            const common::M3F off_R = s.offset_R_L_I.toRotationMatrix().cast<float>();
            const common::V3F off_t = s.offset_T_L_I.cast<float>();
            const common::M3F Rt = s.rot.toRotationMatrix().transpose().cast<float>();

            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                common::V3F point_this_be = corr_pts_[i].head<3>();
                common::M3F point_be_crossmat = SKEW_SYM_MATRIX(point_this_be);
                common::V3F point_this = off_R * point_this_be + off_t;
                common::M3F point_crossmat = SKEW_SYM_MATRIX(point_this);

                /*** get the normal vector of closest surface/corner ***/
                common::V3F norm_vec = corr_norm_[i].head<3>();

                /*** calculate the Measurement Jacobian matrix H ***/
                common::V3F C(Rt * norm_vec);
                common::V3F A(point_crossmat * C);

                if (extrinsic_est_en_) {
                    common::V3F B(point_be_crossmat * off_R.transpose() * C);
                    ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], B[0],
                        B[1], B[2], C[0], C[1], C[2];
                } else {
                    ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0;
                }

                /*** Measurement: distance to the closest surface/corner ***/
                ekfom_data.h(i) = -corr_pts_[i][3];
            });
        },
        "    ObsModel (IEKF Build Jacobian)");
}

/////////////////////////////////////  debug save / show /////////////////////////////////////////////////////
void LaserMapping::PublishState(const state_ikfom& state) const {
    geometry_msgs::msg::Vector3 state_velocity;
    state_velocity.x = state.vel[0];
    state_velocity.y = state.vel[1];
    state_velocity.z = state.vel[2];
    pub_state_velocity_->publish(state_velocity);

    geometry_msgs::msg::Vector3 state_acc_bias;
    state_acc_bias.x = state.ba[0];
    state_acc_bias.y = state.ba[1];
    state_acc_bias.z = state.ba[2];
    pub_state_acc_bias_->publish(state_acc_bias);

    geometry_msgs::msg::Vector3 state_gyr_bias;
    state_gyr_bias.x = state.bg[0];
    state_gyr_bias.y = state.bg[1];
    state_gyr_bias.z = state.bg[2];
    pub_state_gyr_bias_->publish(state_gyr_bias);
}

void LaserMapping::PublishPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path) {
    SetPosestamp(msg_body_pose_);
    msg_body_pose_.header.stamp = common::SecondsToTimestamp(lidar_end_time_);
    msg_body_pose_.header.frame_id = tf_world_frame_;

    /*** if path is too large, the rvis will crash ***/
    path_.poses.push_back(msg_body_pose_);
    if (run_in_offline_ == false) {
        pub_path->publish(path_);
    }
}

void LaserMapping::PublishOdometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_aft_mapped) {
    odom_aft_mapped_.header.frame_id = tf_world_frame_;
    odom_aft_mapped_.child_frame_id = tf_imu_frame_;
    odom_aft_mapped_.header.stamp = common::SecondsToTimestamp(lidar_end_time_);
    SetPosestamp(odom_aft_mapped_.pose);
    pub_odom_aft_mapped->publish(odom_aft_mapped_);
    auto P = kf_.get_P();
    for (int i = 0; i < 6; i++) {
        int k = i < 3 ? i + 3 : i - 3;
        odom_aft_mapped_.pose.covariance[i * 6 + 0] = P(k, 3);
        odom_aft_mapped_.pose.covariance[i * 6 + 1] = P(k, 4);
        odom_aft_mapped_.pose.covariance[i * 6 + 2] = P(k, 5);
        odom_aft_mapped_.pose.covariance[i * 6 + 3] = P(k, 0);
        odom_aft_mapped_.pose.covariance[i * 6 + 4] = P(k, 1);
        odom_aft_mapped_.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    static auto tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom_aft_mapped_.header;
    transform.child_frame_id = tf_imu_frame_;
    transform.transform.translation.x = odom_aft_mapped_.pose.pose.position.x;
    transform.transform.translation.y = odom_aft_mapped_.pose.pose.position.y;
    transform.transform.translation.z = odom_aft_mapped_.pose.pose.position.z;
    transform.transform.rotation = odom_aft_mapped_.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);

    // tf2_ros::Transform transform;
    // tf2_ros::Quaternion q;
    // transform.setOrigin(tf::Vector3(odom_aft_mapped_.pose.pose.position.x, odom_aft_mapped_.pose.pose.position.y,
    //                                 odom_aft_mapped_.pose.pose.position.z));
    // q.setW(odom_aft_mapped_.pose.pose.orientation.w);
    // q.setX(odom_aft_mapped_.pose.pose.orientation.x);
    // q.setY(odom_aft_mapped_.pose.pose.orientation.y);
    // q.setZ(odom_aft_mapped_.pose.pose.orientation.z);
    // transform.setRotation(q);
    // br->sendTransform(tf::StampedTransform(transform, odom_aft_mapped_.header.stamp, tf_world_frame_, tf_imu_frame_));
}

void LaserMapping::PublishFrameWorld() {
    if (!(run_in_offline_ == false && scan_pub_en_) && !pcd_save_en_) {
        return;
    }

    PointCloudType::Ptr laserCloudWorld;
    if (dense_pub_en_) {
        PointCloudType::Ptr laserCloudFullRes(scan_undistort_);
        int size = laserCloudFullRes->points.size();
        laserCloudWorld.reset(new PointCloudType(size, 1));
        for (int i = 0; i < size; i++) {
            PointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
        }
    } else {
        laserCloudWorld = scan_down_world_;
    }

    if (run_in_offline_ == false && scan_pub_en_) {
        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = common::SecondsToTimestamp(lidar_end_time_);
        laserCloudmsg.header.frame_id = tf_world_frame_;
        pub_laser_cloud_world_->publish(laserCloudmsg);
        publish_count_ -= options::PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en_) {
        *pcl_wait_save_ += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num++;
        if (pcl_wait_save_->size() > 0 && pcd_save_interval_ > 0 && scan_wait_num >= pcd_save_interval_) {
            pcd_index_++;
            std::string all_points_dir("PCD/scans_" + std::to_string(pcd_index_) +
                                       std::string(".pcd"));
            pcl::PCDWriter pcd_writer;
            LOG(INFO) << "current scan saved to /PCD/" << all_points_dir;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_);
            pcl_wait_save_->clear();
            scan_wait_num = 0;
        }
    }
}

void LaserMapping::PublishFrameBody(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_body) {
    int size = scan_undistort_->points.size();
    PointCloudType::Ptr laser_cloud_imu_body(new PointCloudType(size, 1));

    for (int i = 0; i < size; i++) {
        PointBodyLidarToIMU(&scan_undistort_->points[i], &laser_cloud_imu_body->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laser_cloud_imu_body, laserCloudmsg);
    laserCloudmsg.header.stamp = common::SecondsToTimestamp(lidar_end_time_);
    laserCloudmsg.header.frame_id = "body";
    pub_laser_cloud_body->publish(laserCloudmsg);
    publish_count_ -= options::PUBFRAME_PERIOD;
}

void LaserMapping::PublishFrameEffectWorld(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_effect_world) {
    int size = corr_pts_.size();
    PointCloudType::Ptr laser_cloud(new PointCloudType(size, 1));

    for (int i = 0; i < size; i++) {
        PointBodyToWorld(corr_pts_[i].head<3>(), &laser_cloud->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laser_cloud, laserCloudmsg);
    laserCloudmsg.header.stamp = common::SecondsToTimestamp(lidar_end_time_);
    laserCloudmsg.header.frame_id = tf_world_frame_;
    pub_laser_cloud_effect_world->publish(laserCloudmsg);
    publish_count_ -= options::PUBFRAME_PERIOD;
}

void LaserMapping::Savetrajectory(const std::string &traj_file) {
    std::ofstream ofs;
    ofs.open(traj_file, std::ios::out);
    if (!ofs.is_open()) {
        LOG(ERROR) << "Failed to open traj_file: " << traj_file;
        return;
    }

    ofs << "#timestamp x y z q_x q_y q_z q_w" << std::endl;
    for (const auto &p : path_.poses) {
        ofs << std::fixed << std::setprecision(6) << common::GetSecondsFromHeader(p.header) << " " << std::setprecision(15)
            << p.pose.position.x << " " << p.pose.position.y << " " << p.pose.position.z << " " << p.pose.orientation.x
            << " " << p.pose.orientation.y << " " << p.pose.orientation.z << " " << p.pose.orientation.w << std::endl;
    }

    ofs.close();
}

///////////////////////////  private method /////////////////////////////////////////////////////////////////////
template <typename T>
void LaserMapping::SetPosestamp(T &out) {
    out.pose.position.x = state_point_.pos(0);
    out.pose.position.y = state_point_.pos(1);
    out.pose.position.z = state_point_.pos(2);
    out.pose.orientation.x = state_point_.rot.coeffs()[0];
    out.pose.orientation.y = state_point_.rot.coeffs()[1];
    out.pose.orientation.z = state_point_.rot.coeffs()[2];
    out.pose.orientation.w = state_point_.rot.coeffs()[3];
}

void LaserMapping::PointBodyToWorld(const PointType *pi, PointType *const po) {
    common::V3D p_body(pi->x, pi->y, pi->z);
    common::V3D p_global(state_point_.rot * (state_point_.offset_R_L_I * p_body + state_point_.offset_T_L_I) +
                         state_point_.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void LaserMapping::PointBodyToWorld(const common::V3F &pi, PointType *const po) {
    common::V3D p_body(pi.x(), pi.y(), pi.z());
    common::V3D p_global(state_point_.rot * (state_point_.offset_R_L_I * p_body + state_point_.offset_T_L_I) +
                         state_point_.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = std::abs(po->z);
}

void LaserMapping::PointBodyLidarToIMU(PointType const *const pi, PointType *const po) {
    common::V3D p_body_lidar(pi->x, pi->y, pi->z);
    common::V3D p_body_imu(state_point_.offset_R_L_I * p_body_lidar + state_point_.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void LaserMapping::Finish() {
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save_->size() > 0 && pcd_save_en_) {
        std::string file_name = std::string("scans.pcd");
        std::string all_points_dir("PCD/" + file_name);
        pcl::PCDWriter pcd_writer;
        LOG(INFO) << "current scan saved to " << all_points_dir;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_);
    }

    LOG(INFO) << "finish done";
}
}  // namespace faster_lio