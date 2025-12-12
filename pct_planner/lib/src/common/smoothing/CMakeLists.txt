set(osqp_DIR "/media/alex/586ba648-3d8f-40ef-96c9-bf6105f008122/home/alex/ros2_ws/3rdparty/osqp-1.0.0/lib/cmake/osqp")

find_package(osqp REQUIRED)

add_library(common_smoothing SHARED
    affine_constraint.cc
    discrete_points_math.cc
    osqp_spline1d_solver.cc
    osqp_spline2d_solver.cc
    polynomialxd.cc
    spline1d_constraint.cc
    spline1d_kernel_helper.cc
    spline1d_kernel.cc
    spline1d_seg.cc
    spline1d.cc
    spline2d_constraint.cc
    spline2d_kernel.cc
    spline2d_seg.cc
    spline2d.cc
    solver/osqp/osqp_interface.cc
    utils/math.cc
)
target_include_directories(common_smoothing PUBLIC
  "/media/alex/586ba648-3d8f-40ef-96c9-bf6105f008122/home/alex/ros2_ws/3rdparty/osqp-1.0.0/include"
)
target_link_libraries(common_smoothing
  osqp::osqpstatic
)