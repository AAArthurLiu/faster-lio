cc_library(
    name = "pcl_conversions",
    hdrs = glob([
        "include/pcl_conversions/**/*.h",
    ]),
    includes = ["include/pcl_conversions"],
    visibility = ["//visibility:public"],
    deps = [
        "@ros2_rclcpp//:rclcpp",
        "@ros2_pcl_msgs//:pcl_msgs",
        "@ros2_message_filters//:message_filters",
        "@ros2_common_interfaces//:cpp_std_msgs",
        "@ros2_common_interfaces//:cpp_sensor_msgs",
    ],
)
