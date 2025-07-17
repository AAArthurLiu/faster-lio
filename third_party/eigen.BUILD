"""BUILD file for Eigen3 library"""

load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "eigen",
    hdrs = glob([
        "Eigen/**",
        "unsupported/**",
    ]),
    includes = ["."],
    visibility = ["//visibility:public"],
)
