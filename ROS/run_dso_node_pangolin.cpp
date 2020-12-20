#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include "cv_bridge/cv_bridge.h"

#include <thread>
#include <cstdio>
#include <unistd.h>

#include <glog/logging.h>

#include "frontend/FullSystem.h"
#include "DatasetReader.h"

using namespace std;
using namespace ldso;

/*********************************************************************************
 * This program demonstrates how to run LDSO in EUROC dataset
 * LDSO currently works on MAV_01-05 and V101, V102, V201
 * Note we don't have photometric calibration in EUROC so we should let the algorithm
 * estimate lighting parameter a,b for us
 *
 * You'd better set a start index greater than zero since DSO does not work well on blurred images
 *
 * Please specify the dataset directory below or by command line parameters
 *********************************************************************************/

std::string output_file = "./results.txt";
std::string calib = "/home/chrisliu/ROSws/ldso_ws/src/ldso_ros/config/camera.txt";
std::string vocPath = "/home/chrisliu/ROSws/ldso_ws/src/ldso_ros/vocab/orbvoc.dbow3";

int startIdx = 0;
int endIdx = 100000;

double rescale = 1;
bool reversePlay = false;
float playbackSpeed = 0;    // 0 for linearize (play as fast as possible, while sequentializing tracking & mapping). otherwise, factor on timestamps.
bool preload = false;

shared_ptr<PangolinDSOViewer> viewer;
shared_ptr<ORBVocabulary> voc;
Undistort *undistort;

shared_ptr<FullSystem> fullSystem;
int frameID = 0;
void vidCb(const sensor_msgs::ImageConstPtr img)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    assert(cv_ptr->image.type() == CV_8U);
    assert(cv_ptr->image.channels() == 1);

    MinimalImageB minImg((int)cv_ptr->image.cols, (int)cv_ptr->image.rows,(unsigned char*)cv_ptr->image.data);
    ImageAndExposure* undistImg = undistort->undistort<unsigned char>(&minImg, 1,0, 1.0f);
    undistImg->timestamp=img->header.stamp.toSec(); // relay the timestamp to dso

    bool skipFrame = false;
    if (ros::Time::now().toSec() > img->header.stamp.toSec() + 0.5 + 0.1 * (frameID % 2)) {
        ROS_ERROR("SKIPFRAME %d (play at %f, now it is %f)!\n", frameID, img->header.stamp.toSec(), ros::Time::now().toSec());
        skipFrame = true;
    }

    if (!skipFrame)
        fullSystem->addActiveFrame(undistImg, frameID);

    frameID++;
    delete undistImg;

    if (setting_fullResetRequested) {
        LOG(INFO) << "RESETTING!";
        fullSystem = shared_ptr<FullSystem>(new FullSystem(voc));
        float *gamma;
        if (undistort == 0 || undistort->photometricUndist == 0)
        {
            gamma = 0;
        }
        else
            gamma = undistort->photometricUndist->getG();
        fullSystem->setGammaFunction(gamma);
        fullSystem->linearizeOperation = (playbackSpeed == 0);
        if (viewer) {
            viewer->reset();
            sleep(1);
            fullSystem->setViewer(viewer);
        }
        setting_fullResetRequested = false;
    }
    if (fullSystem->isLost) {
        LOG(INFO) << "Lost!";
        return;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "dso_node");

    FLAGS_colorlogtostderr = true;
    setting_maxAffineWeight = 0.1;  // don't use affine brightness weight in Euroc!

    // EuRoC has no photometric calibration
    printf("PHOTOMETRIC MODE WITHOUT CALIBRATION!\n");
    setting_photometricCalibration = 0;
    setting_affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
    setting_affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).

    undistort = Undistort::getUndistorterForFile(calib, "", "");

    int w_out, h_out;
    Eigen::Matrix3f K;
    K = undistort->getK().cast<float>();
    w_out = undistort->getSize()[0];
    h_out = undistort->getSize()[1];
    setGlobalCalib(w_out, h_out, K);

    voc = shared_ptr<ORBVocabulary>(new ORBVocabulary());
    voc->load(vocPath);

    fullSystem  = shared_ptr<FullSystem>(new FullSystem(voc));
    float *gamma;
    if (undistort == 0 || undistort->photometricUndist == 0)
    {
        gamma = 0;
    }
    else
        gamma = undistort->photometricUndist->getG();
    fullSystem->setGammaFunction(gamma);
    fullSystem->linearizeOperation = (playbackSpeed == 0);

    viewer = nullptr;
    if (!disableAllDisplay) {
        viewer = shared_ptr<PangolinDSOViewer>(new PangolinDSOViewer(wG[0], hG[0], false));
        fullSystem->setViewer(viewer);
    } else {
        LOG(INFO) << "visualization is diabled!" << endl;
    }

    ros::NodeHandle nh;
    ros::Subscriber imgSub = nh.subscribe("/camera/color/image_raw", 1, &vidCb);

    std::thread runthread([&]() {
        if (viewer)
            viewer->run();  // mac os should keep this in main thread.
    });

    ros::spin();

//    fullSystem->printResult(output_file, true);
//    fullSystem->printResult(output_file + ".noloop", false);

    runthread.join();


    LOG(INFO) << "EXIT NOW!";
    return 0;
}
