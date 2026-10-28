# ROS-free build of SVO Pro's monocular/stereo frontend and its Ceres VIO backend,
# without catkin: one shared library, libsvo_pro_open.so, from the core packages below.
# The catkin packages (and their own CMakeLists.txt) are untouched; svo_ros, rqt_svo,
# svo_pgo, svo_online_loopclosing, svo_global_map and svo_benchmarking are not built
# here, so loop closing and the global map are compiled out.
#
#   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
#         -DCMAKE_PREFIX_PATH=<opengv prefix> \
#         -DSVO_FAST_DIR=<uzh-rpg/fast_neon checkout> \
#         -DSVO_MINKINDR_INCLUDE_DIR=<the directory holding minkindr's kindr/ headers>
#   cmake --build build -j8
#
# Differences from svo_cmake/cmake/Modules/SvoSetup.cmake: C++17 (Ceres >= 2.1), no
# -march=native (an Eigen ABI shared with whatever links this library), no -Werror,
# no -fsee (not a GCC flag), SVO_USE_ROS/SVO_LOOP_CLOSING undefined. -ffast-math and
# friends are compile options only, so libsvo_pro_open.so doesn't pull crtfastmath.o
# (process-wide flush-to-zero) into its users.
cmake_minimum_required(VERSION 3.16)
project(svo_pro_open LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(SVO_FAST_DIR "" CACHE PATH "uzh-rpg/fast_neon checkout")
set(SVO_MINKINDR_INCLUDE_DIR "" CACHE PATH "Directory holding ethz-asl/minkindr's kindr/ headers")
foreach(dir SVO_FAST_DIR SVO_MINKINDR_INCLUDE_DIR)
  if(NOT EXISTS "${${dir}}")
    message(FATAL_ERROR "${dir} (${${dir}}) does not exist - see the header of this file")
  endif()
endforeach()

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED COMPONENTS core imgproc features2d calib3d video highgui)
find_package(Ceres 2.1 REQUIRED)
find_package(opengv REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(glog REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenMP)

set(SVO_WARNINGS
  -Wall -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas
  -Wno-unused-but-set-parameter -Wno-int-in-bool-context -Wno-maybe-uninitialized
  -Wno-unused-function -Wno-deprecated-declarations -Wno-class-memaccess
  -Wno-sign-compare -Wno-reorder -Wno-misleading-indentation)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
  set(SVO_ARCH_FLAGS -mmmx -msse -msse2 -msse3 -mssse3 -mno-avx)
else()
  set(SVO_ARCH_FLAGS)
endif()
set(SVO_RELEASE_FLAGS
  -O3 -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops
  -ffast-math -fno-finite-math-only)
if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
  # Eigen's NEON sign flips (quaternion conjugate/inverse) XOR with a -0.0 mask that
  # -fno-signed-zeros folds to +0.0: q.inverse() becomes q and the pose Minus breaks
  # (svo_manifold_check). x86's SSE masks are integer bit patterns, unaffected.
  list(APPEND SVO_RELEASE_FLAGS -fsigned-zeros)
endif()

# FAST corner detector, linked in statically. fast_neon's SSE path on x86_64; elsewhere
# (aarch64) the plain C++ FAST-10 - its NEON file wants armv7's __ARM_NEON__.
set(SVO_FAST_SOURCES
  ${SVO_FAST_DIR}/src/fast_9.cpp
  ${SVO_FAST_DIR}/src/fast_10.cpp
  ${SVO_FAST_DIR}/src/fast_10_score.cpp
  ${SVO_FAST_DIR}/src/nonmax_3x3.cpp)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
  list(APPEND SVO_FAST_SOURCES ${SVO_FAST_DIR}/src/faster_corner_10_sse.cpp)
endif()
add_library(svo_fast STATIC ${SVO_FAST_SOURCES})
target_include_directories(svo_fast PUBLIC ${SVO_FAST_DIR}/include)
target_compile_options(svo_fast PRIVATE -O3 ${SVO_ARCH_FLAGS} -fomit-frame-pointer -Wno-unused-variable)

set(SVO_PACKAGES
  rpg_common vikit/vikit_common vikit/vikit_cameras vikit/vikit_solver svo_common
  svo_direct svo_tracker svo_img_align svo_vio_common svo svo_ceres_backend)
set(SVO_SOURCES
  rpg_common/src/fs.cpp
  vikit/vikit_common/src/homography.cpp
  vikit/vikit_common/src/math_utils.cpp
  vikit/vikit_common/src/performance_monitor.cpp
  vikit/vikit_common/src/sample.cpp
  vikit/vikit_common/src/user_input_thread.cpp
  vikit/vikit_common/src/vision.cpp
  vikit/vikit_cameras/src/camera_factory.cpp
  vikit/vikit_cameras/src/ncamera.cpp
  vikit/vikit_cameras/src/camera_geometry_base.cpp
  vikit/vikit_cameras/src/camera_yaml_serialization.cpp
  vikit/vikit_cameras/src/equidistant_fisheye_geometry.cpp
  vikit/vikit_cameras/src/equidistant_fisheye_projection.cpp
  vikit/vikit_cameras/src/omni_geometry.cpp
  vikit/vikit_cameras/src/omni_projection.cpp
  vikit/vikit_cameras/src/ncamera_yaml_serialization.cpp
  vikit/vikit_solver/src/robust_cost.cpp
  svo_common/src/frame.cpp
  svo_common/src/point.cpp
  svo_direct/src/patch_warp.cpp
  svo_direct/src/feature_alignment.cpp
  svo_direct/src/feature_detection.cpp
  svo_direct/src/feature_detection_utils.cpp
  svo_direct/src/matcher.cpp
  svo_direct/src/depth_filter.cpp
  svo_direct/src/depth_estimation.cpp
  svo_direct/src/elder_zucker.cpp
  svo_tracker/src/feature_tracking_types.cpp
  svo_tracker/src/feature_tracking_utils.cpp
  svo_tracker/src/feature_tracking_viz.cpp
  svo_tracker/src/feature_tracker.cpp
  svo_img_align/src/sparse_img_align_base.cpp
  svo_img_align/src/sparse_img_align.cpp
  svo_vio_common/src/vio_common.cpp
  svo_vio_common/src/test_utils.cpp
  svo/src/frame_handler_mono.cpp
  svo/src/frame_handler_stereo.cpp
  svo/src/frame_handler_array.cpp
  svo/src/frame_handler_base.cpp
  svo/src/map.cpp
  svo/src/pose_optimizer.cpp
  svo/src/initialization.cpp
  svo/src/reprojector.cpp
  svo/src/imu_handler.cpp
  svo/src/stereo_triangulation.cpp
  svo/src/io.cpp
  svo_ceres_backend/src/ceres_backend_interface.cpp
  svo_ceres_backend/src/estimator.cpp
  svo_ceres_backend/src/error_interface.cpp
  svo_ceres_backend/src/homogeneous_point_error.cpp
  svo_ceres_backend/src/homogeneous_point_parameter_block.cpp
  svo_ceres_backend/src/homogeneous_point_local_parameterization.cpp
  svo_ceres_backend/src/imu_error.cpp
  svo_ceres_backend/src/local_parameterization_additional_interfaces.cpp
  svo_ceres_backend/src/map.cpp
  svo_ceres_backend/src/marginalization_error.cpp
  svo_ceres_backend/src/pose_error.cpp
  svo_ceres_backend/src/pose_parameter_block.cpp
  svo_ceres_backend/src/pose_local_parameterization.cpp
  svo_ceres_backend/src/relative_pose_error.cpp
  svo_ceres_backend/src/speed_and_bias_error.cpp
  svo_ceres_backend/src/speed_and_bias_parameter_block.cpp
  svo_ceres_backend/src/motion_detector.cpp
  svo_ceres_backend/src/outlier_rejection.cpp
  svo_ceres_backend/src/map_alignment.cpp)

add_library(svo_pro_open SHARED ${SVO_SOURCES})
foreach(pkg ${SVO_PACKAGES})
  target_include_directories(svo_pro_open PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${pkg}/include>)
endforeach()
target_include_directories(svo_pro_open PUBLIC
  $<BUILD_INTERFACE:${SVO_MINKINDR_INCLUDE_DIR}>)
target_compile_definitions(svo_pro_open PUBLIC
  SVO_USE_OPENGV SVO_DEPTHFILTER_IN_REPROJECTOR _LINUX _REENTRANT)
target_compile_options(svo_pro_open PRIVATE
  ${SVO_WARNINGS} ${SVO_ARCH_FLAGS} $<$<CONFIG:Release>:${SVO_RELEASE_FLAGS}>)
if(OpenMP_CXX_FOUND)
  target_compile_definitions(svo_pro_open PRIVATE USE_OPENMP)
  target_link_libraries(svo_pro_open PRIVATE OpenMP::OpenMP_CXX)
endif()
target_link_libraries(svo_pro_open
  PUBLIC Eigen3::Eigen ${OpenCV_LIBS} Ceres::ceres glog::glog yaml-cpp Boost::boost
  PRIVATE svo_fast opengv pthread)

# Numerical self-check of the Ceres Manifold port (svo_ceres_backend).
add_executable(svo_manifold_check svo_ceres_backend/test/manifold_check.cpp)
target_link_libraries(svo_manifold_check svo_pro_open)
target_compile_options(svo_manifold_check PRIVATE ${SVO_ARCH_FLAGS})
