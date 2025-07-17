#include "pointcloud_preprocess.h"

#include <glog/logging.h>
#include <execution>

namespace faster_lio {

std::string ToString(LidarType type) {
    switch (type) {
        case LidarType::AVIA:
            return "AVIA";
        case LidarType::VELO32:
            return "VELO32";
        case LidarType::OUST64:
            return "OUST64";
        case LidarType::JT16:
            return "JT16";
        case LidarType::MID360:
            return "MID360";
        default:
            LOG(FATAL) << "ToString() for type " << static_cast<int>(type) << " is NOT implemented. ";
            return "UNKNOWN";
    }
}

void PointCloudPreprocess::SetLidarType(int lidar_type_int) {
    LidarType lidar_type = static_cast<LidarType>(lidar_type_int);
    SetLidarType(lidar_type);
}

void PointCloudPreprocess::SetLidarType(LidarType lidar_type) {
    LOG(INFO) << "Using lidar type: " << ToString(lidar_type);
    lidar_type_ = lidar_type;
}

void PointCloudPreprocess::Process(const faster_lio_interfaces::msg::CustomMsg::ConstSharedPtr &msg,
                                   PointCloudType::Ptr &pcl_out) {
    AviaHandler(msg);
    *pcl_out = cloud_out_;
}

void PointCloudPreprocess::Process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
                                   PointCloudType::Ptr &pcl_out) {
    switch (lidar_type_) {
        case LidarType::AVIA:
            LOG(FATAL) << "AVIA LiDAR should be handled somewhere else";
        case LidarType::OUST64:
            Oust64Handler(msg);
            break;
        case LidarType::VELO32:
            VelodyneHandler(msg);
            break;
        case LidarType::JT16:
            TimedPointcloudHandler(msg);
            break;
        case LidarType::MID360:
            TimedPointcloudHandler(msg);
            break;
        default:
            LOG(ERROR) << "Error LiDAR Type";
            break;
    }
    *pcl_out = cloud_out_;
}

void PointCloudPreprocess::AviaHandler(const faster_lio_interfaces::msg::CustomMsg::ConstSharedPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();
    uint plsize = static_cast<uint>(msg->point_num);
    if (plsize < 1) {
        LOG(WARNING) << "Point cloud size is less than 1, skipping processing.";
        return;
    }

    cloud_out_.reserve(plsize);
    cloud_full_.resize(plsize);

    std::vector<bool> is_valid_pt(plsize, false);
    std::vector<uint> index(plsize - 1);
    for (uint i = 0; i < plsize - 1; ++i) {
        index[i] = i + 1;  // 从1开始
    }

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const uint &i) {
        if ((msg->points[i].line < num_scans_) &&
            ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)) {
            if (i % point_filter_num_ == 0) {
                cloud_full_[i].x = msg->points[i].x;
                cloud_full_[i].y = msg->points[i].y;
                cloud_full_[i].z = msg->points[i].z;
                cloud_full_[i].intensity = msg->points[i].reflectivity;
                cloud_full_[i].curvature =
                    msg->points[i].offset_time /
                    float(1000000);  // use curvature as time of each laser points, curvature unit: ms

                if (((abs(cloud_full_[i].x - cloud_full_[i - 1].x) > 1e-7) ||
                     (abs(cloud_full_[i].y - cloud_full_[i - 1].y) > 1e-7) ||
                     (abs(cloud_full_[i].z - cloud_full_[i - 1].z) > 1e-7)) &&
                    (cloud_full_[i].x * cloud_full_[i].x + cloud_full_[i].y * cloud_full_[i].y +
                         cloud_full_[i].z * cloud_full_[i].z >
                     (blind_ * blind_))) {
                    is_valid_pt[i] = true;
                }
            }
        }
    });

    for (uint i = 1; i < plsize; i++) {
        if (is_valid_pt[i]) {
            cloud_out_.points.push_back(cloud_full_[i]);
        }
    }
}

void PointCloudPreprocess::Oust64Handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.size();
    cloud_out_.reserve(plsize);

    for (size_t i = 0; i < pl_orig.points.size(); i++) {
        if (i % point_filter_num_ != 0) continue;

        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;

        if (range < (blind_ * blind_)) continue;

        Eigen::Vector3d pt_vec;
        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.curvature = pl_orig.points[i].t / 1e6;  // curvature unit: ms

        cloud_out_.points.push_back(added_pt);
    }
}

void PointCloudPreprocess::VelodyneHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    cloud_out_.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 3.61;  // scan angular velocity
    std::vector<bool> is_first(num_scans_, true);
    std::vector<double> yaw_fp(num_scans_, 0.0);    // yaw of first scan point
    std::vector<float> yaw_last(num_scans_, 0.0);   // yaw of last scan point
    std::vector<float> time_last(num_scans_, 0.0);  // last offset time
    /*****************************************************************/

    if (pl_orig.points[plsize - 1].time > 0) {
        given_offset_time_ = true;
    } else {
        given_offset_time_ = false;
        double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#endif
        double yaw_end = yaw_first;
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
        int layer_first = pl_orig.points[0].ring;
        for (uint i = plsize - 1; i > 0; i--) {
            if (pl_orig.points[i].ring == layer_first) {
                yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
                break;
            }
        }
    }

    for (int i = 0; i < plsize; i++) {
        PointType added_pt;

        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.curvature = pl_orig.points[i].time * time_scale_;  // curvature unit: ms

        if (!given_offset_time_) {
            int layer = pl_orig.points[i].ring;
            double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

            if (is_first[layer]) {
                yaw_fp[layer] = yaw_angle;
                is_first[layer] = false;
                added_pt.curvature = 0.0;
                yaw_last[layer] = yaw_angle;
                time_last[layer] = added_pt.curvature;
                continue;
            }

            // compute offset time
            if (yaw_angle <= yaw_fp[layer]) {
                added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
            } else {
                added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
            }

            if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

            yaw_last[layer] = yaw_angle;
            time_last[layer] = added_pt.curvature;
        }

        if (i % point_filter_num_ == 0) {
            if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind_ * blind_)) {
                cloud_out_.points.push_back(added_pt);
            }
        }
    }
}

void PointCloudPreprocess::TimedPointcloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();

    // Ensure fields: x, y, z, intensity, timestamp
    const auto &fields = msg->fields;
    int offset_x = -1, offset_y = -1, offset_z = -1, offset_intensity = -1, offset_time = -1;
    for (const auto &field : fields) {
        if (field.name == "x") {
            offset_x = field.offset;
        } else if (field.name == "y") {
            offset_y = field.offset;
        } else if (field.name == "z") {
            offset_z = field.offset;
        } else if (field.name == "intensity") {
            offset_intensity = field.offset;
        } else if (field.name == "timestamp") {
            offset_time = field.offset;
            if (field.datatype != 8) {
                LOG(ERROR) << "Expected timestamp datatype 8 (float64), got " << field.datatype;
                return;
            }
        }
    }

    if (offset_x < 0 || offset_y < 0 || offset_z < 0 || offset_intensity < 0 || offset_time < 0) {
        LOG(ERROR) << "Missing required fields in timed point cloud - x:" << offset_x << " y:" << offset_y
                   << " z:" << offset_z << " intensity:" << offset_intensity << " time:" << offset_time;
        return;
    }

    const size_t point_step = msg->point_step;
    const size_t num_points = msg->width * msg->height;
    cloud_out_.reserve(num_points / point_filter_num_ + 1);

    const uint8_t *data_ptr = msg->data.data();
    const double time_ratio_to_second = lidar_type_ == LidarType::MID360 ? 1.0e-9 : 1.0;  // MID360 uses nanoseconds
    double head_time = *reinterpret_cast<const double *>(data_ptr + offset_time) * time_ratio_to_second;

    int points_added = 0;
    int points_filtered_by_step = 0;
    int points_filtered_by_blind = 0;

    for (size_t i = 0; i < num_points; ++i) {
        if (i % point_filter_num_ != 0) {
            points_filtered_by_step++;
            continue;
        }

        const uint8_t *pt_base = data_ptr + i * point_step;

        float x = *reinterpret_cast<const float *>(pt_base + offset_x);
        float y = *reinterpret_cast<const float *>(pt_base + offset_y);
        float z = *reinterpret_cast<const float *>(pt_base + offset_z);
        double range = x * x + y * y + z * z;
        if (range < blind_ * blind_) {
            points_filtered_by_blind++;
            continue;
        }

        float intensity = *reinterpret_cast<const float *>(pt_base + offset_intensity);
        double time = *reinterpret_cast<const double *>(pt_base + offset_time) * time_ratio_to_second;

        PointType added_pt;
        added_pt.x = x;
        added_pt.y = y;
        added_pt.z = z;
        added_pt.intensity = intensity;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.curvature = static_cast<float>((time - head_time) * 1000.0);  // convert to milliseconds

        cloud_out_.push_back(added_pt);
        points_added++;
    }

    LOG_EVERY_N(INFO, 100) << "TimedPointcloudHandler results: " << points_added << " points added, "
                           << points_filtered_by_step << " filtered by step, " << points_filtered_by_blind
                           << " filtered by blind distance";
}

}  // namespace faster_lio
