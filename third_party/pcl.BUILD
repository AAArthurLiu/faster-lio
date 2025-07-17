package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pcl",
    hdrs = glob([
        "pcl/**/*.h",
        "pcl/**/*.hpp",
    ]),
    linkopts = [
        "-L/usr/lib/x86_64-linux-gnu",
        "-lpcl_common",
        "-lpcl_io",
        "-lpcl_filters",
        "-lpcl_features", 
        "-lpcl_kdtree",
        "-lpcl_sample_consensus",
        "-lpcl_search",
        "-lpcl_segmentation",
        "-lpcl_surface",
        "-lpcl_registration",
        "-lpcl_visualization",
    ],
    deps = [
        "@com_gitlab_eigen//:eigen",
    ],
)
