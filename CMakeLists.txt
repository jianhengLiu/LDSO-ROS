cmake_minimum_required(VERSION 2.8)
project(ldso_ros)

# Set default build type if not specified otherwise.
# See https://cmake.org/pipermail/cmake/2012-May/050243.html
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    message(STATUS "Setting build type to '${CMAKE_BUILD_TYPE}' as none was specified.")
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
            "MinSizeRel" "RelWithDebInfo")
endif()

# C++ standard
if(${CMAKE_VERSION} VERSION_LESS "3.2.0")
    set( CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}" )
else()
    set(CMAKE_CXX_STANDARD 11)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
endif()

# costum modules
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslib
        sensor_msgs
        cv_bridge
        )

#find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread)
find_package(Pangolin REQUIRED)

find_package (glog 0.4.0 REQUIRED)

# optional libs
find_package(LibZip QUIET)

set(CMAKE_CXX_FLAGS "-Wall -Wno-deprecated -march=native -Wno-duplicate-decl-specifier -Wno-ignored-qualifiers -Wno-reorder -Wno-missing-braces")

if(NOT APPLE)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif()

# decide if we have ziplib.
if (LIBZIP_LIBRARY)
    message("--- found ziplib (${LIBZIP_VERSION}), compiling with zip capability.")
    add_definitions(-DHAS_ZIPLIB=1)
    include_directories( ${LIBZIP_INCLUDE_DIR_ZIP} ${LIBZIP_INCLUDE_DIR_ZIPCONF} )
else()
    message("--- not found ziplib (${LIBZIP_LIBRARY}), compiling without zip capability.")
    set(LIBZIP_LIBRARY "")
endif()


catkin_package()

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -mavx")

include_directories(
        ${catkin_INCLUDE_DIRS}

        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${Glog_INCLUDE_DIRS}

#        ${PROJECT_SOURCE_DIR}/src/
        ${PROJECT_SOURCE_DIR}/include/
#        ${PROJECT_SOURCE_DIR}/examples/
        ${PROJECT_SOURCE_DIR}/thirdparty/
#        ${PROJECT_SOURCE_DIR}/thirdparty/sophus/
        ${PROJECT_SOURCE_DIR}/thirdparty/DBoW3/
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/
)

set( THIRD_PARTY_LIBS
        ${catkin_LIBRARIES}

        ${OpenCV_LIBS}
        glog::glog
        ${LIBZIP_LIBRARY}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o${CMAKE_SHARED_LIBRARY_SUFFIX}
        ${PROJECT_SOURCE_DIR}/thirdparty/DBoW3/build/src/libDBoW3${CMAKE_SHARED_LIBRARY_SUFFIX}
        )

set( SOURCE_FILES
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
        )

#add_subdirectory(src)
#add_subdirectory(examples)

add_executable(run_dso_tum_mono
        examples/run_dso_tum_mono.cc
        examples/DatasetReader.h
        ${SOURCE_FILES}
        ${HEADER_FILES}
        )
#add_dependencies(run_dso_tum_mono ${catkin_EXPORTED_TARGETS})
target_link_libraries(run_dso_tum_mono ${THIRD_PARTY_LIBS})

add_executable(run_dso_euroc
        examples/run_dso_euroc.cc
        examples/DatasetReader.h
        ${SOURCE_FILES}
        ${HEADER_FILES}
        )
#add_dependencies(run_dso_euroc ${catkin_EXPORTED_TARGETS})
target_link_libraries(run_dso_euroc ${THIRD_PARTY_LIBS})

add_executable(run_dso_kitti
        examples/run_dso_kitti.cc
        examples/DatasetReader.h
        ${SOURCE_FILES}
        ${HEADER_FILES}
        )
#add_dependencies(run_dso_kitti ${catkin_EXPORTED_TARGETS})
target_link_libraries(run_dso_kitti ${THIRD_PARTY_LIBS})

#add_executable(run_dso_node
#        ROS/run_dso_node.cpp
#        ${SOURCE_FILES}
#        ${HEADER_FILES}
#        )
#target_link_libraries(run_dso_node ${THIRD_PARTY_LIBS})
