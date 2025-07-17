cc_library(
    name = "ros2_tf2",
    hdrs = glob([
        "include/tf2_*/**/*.hpp",
        "include/tf2_*/**/*.h",
        "include/tf2/**/*.hpp",
        "include/tf2/**/*.h",
    ]),
    srcs = glob([
        "lib/libtf2_*.so",
    ]),
    includes = [
        "include/tf2",
        "include/tf2_eigen",
        "include/tf2_msgs",
        "include/tf2_ros",
    ],
    visibility = ["//visibility:public"],
    deps = [],
)
