cmake_minimum_required(VERSION 2.4.6)
project(ldso_ros)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries


find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslib
        sensor_msgs
        cv_bridge
        )

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread)
find_package(Pangolin REQUIRED)

find_package (glog 0.4.0 REQUIRED)

catkin_package()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -mavx")

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/src/
        ${PROJECT_SOURCE_DIR}/include/
        ${PROJECT_SOURCE_DIR}/examples/
        ${PROJECT_SOURCE_DIR}/thirdparty/
        ${PROJECT_SOURCE_DIR}/thirdparty/sophus/
        ${PROJECT_SOURCE_DIR}/thirdparty/DBoW3/
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
#        ${Glog_INCLUDE_DIRS}
#        ${PCL_INCLUDE_DIRS}
)

set( THIRD_PARTY_LIBS
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        glog::glog
        ${LIBZIP_LIBRARY}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o${CMAKE_SHARED_LIBRARY_SUFFIX}
        ${PROJECT_SOURCE_DIR}/thirdparty/DBoW3/build/src/libDBoW3${CMAKE_SHARED_LIBRARY_SUFFIX}
#        ${PCL_LIBRARIES}
        )

set( SOURCE_FILES
        examples/run_dso_tum_mono.cc

        src/Feature.cc
        src/Frame.cc
        src/Point.cc
        src/Setting.cc
        src/Camera.cc
        src/Map.cc

        src/internal/PointHessian.cc
        src/internal/FrameHessian.cc
        src/internal/GlobalCalib.cc
        src/internal/FrameFramePrecalc.cc
        src/internal/Residuals.cc
        src/internal/ImmaturePoint.cc
        src/internal/PR.cc

        src/internal/OptimizationBackend/AccumulatedSCHessian.cc
        src/internal/OptimizationBackend/AccumulatedTopHessian.cc
        src/internal/OptimizationBackend/EnergyFunctional.cc

        src/frontend/CoarseTracker.cc
        src/frontend/CoarseInitializer.cc
        src/frontend/FullSystem.cc
        src/frontend/DSOViewer.cc
        src/frontend/FeatureDetector.cc
        src/frontend/FeatureMatcher.cc
        src/frontend/LoopClosing.cc
        src/frontend/PixelSelector2.cc
        src/frontend/Undistort.cc
        src/frontend/ImageRW_OpenCV.cc
        )

set( HEADER_FILES
        include/Feature.h
        include/Frame.h
        include/Point.h
        include/Settings.h
        include/Camera.h
        include/Map.h
        include/AffLight.h
        include/NumTypes.h

        include/internal/PointHessian.h
        include/internal/FrameHessian.h
        include/internal/GlobalCalib.h
        include/internal/GlobalFuncs.h
        include/internal/CalibHessian.h
        include/internal/FrameFramePrecalc.h
        include/internal/Residuals.h
        include/internal/RawResidualJacobian.h
        include/internal/ResidualProjections.h
        include/internal/ImmaturePoint.h
        include/internal/PR.h
        include/internal/IndexThreadReduce.h

        include/internal/OptimizationBackend/AccumulatedSCHessian.h
        include/internal/OptimizationBackend/AccumulatedTopHessian.h
        include/internal/OptimizationBackend/EnergyFunctional.h
        include/internal/OptimizationBackend/MatrixAccumulators.h

        include/frontend/CoarseTracker.h
        include/frontend/CoarseInitializer.h
        include/frontend/DSOViewer.h
        include/frontend/FeatureDetector.h
        include/frontend/FeatureMatcher.h
        include/frontend/FullSystem.h
        include/frontend/ImageAndExposure.h
        include/frontend/ImageRW.h
        include/frontend/LoopClosing.h
        include/frontend/MinimalImage.h
        include/frontend/nanoflann.h
        include/frontend/PixelSelector2.h
        include/frontend/Undistort.h

        examples/DatasetReader.h
        )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(dso_slam ${SOURCE_FILES} ${HEADER_FILES})
add_dependencies(dso_slam ${catkin_EXPORTED_TARGETS})
target_link_libraries(dso_slam ${THIRD_PARTY_LIBS})