#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include "cv_bridge/cv_bridge.h"

#include <thread>
#include <cstdio>
#include <unistd.h>

#include <glog/logging.h>

#include "frontend/FullSystem.h"
#include "frontend/Undistort.h"
//#include "DatasetReader.h"

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

void settingsDefault(int preset) {
    printf("\n=============== PRESET Settings: ===============\n");
    if (preset == 0 || preset == 1) {
        printf("DEFAULT settings:\n"
               "- %s real-time enforcing\n"
               "- 2000 active points\n"
               "- 5-7 active frames\n"
               "- 1-6 LM iteration each KF\n"
               "- original image resolution\n", preset == 0 ? "no " : "1x");

        playbackSpeed = (preset == 0 ? 0 : 1);
        preload = preset == 1;
        setting_desiredImmatureDensity = 1500;
        setting_desiredPointDensity = 2000;
        setting_minFrames = 5;
        setting_maxFrames = 7;
        setting_maxOptIterations = 6;
        setting_minOptIterations = 1;

        setting_logStuff = false;
    }

    if (preset == 2 || preset == 3) {
        printf("FAST settings:\n"
               "- %s real-time enforcing\n"
               "- 800 active points\n"
               "- 4-6 active frames\n"
               "- 1-4 LM iteration each KF\n"
               "- 424 x 320 image resolution\n",
               preset == 2 ? "no " : "5x");

        playbackSpeed = (preset == 2 ? 0 : 5);
        preload = preset == 3;
        setting_desiredImmatureDensity = 600;
        setting_desiredPointDensity = 800;
        setting_minFrames = 4;
        setting_maxFrames = 6;
        setting_maxOptIterations = 4;
        setting_minOptIterations = 1;

        benchmarkSetting_width = 424;
        benchmarkSetting_height = 320;

        setting_logStuff = false;
    }

    printf("==============================================\n");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ldso_node");

//    settings:
    {
        int option = 1;
        if (option == 0) {
            printf("PHOTOMETRIC MODE WITH CALIBRATION!\n");
        } else if (option == 1) {
            printf("PHOTOMETRIC MODE WITHOUT CALIBRATION!\n");
            setting_photometricCalibration = 0;
            setting_affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
            setting_affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).
        } else if (option == 2) {
            printf("PHOTOMETRIC MODE WITH PERFECT IMAGES!\n");
            setting_photometricCalibration = 0;
            setting_affineOptModeA = -1; //-1: fix. >=0: optimize (with prior, if > 0).
            setting_affineOptModeB = -1; //-1: fix. >=0: optimize (with prior, if > 0).
            setting_minGradHistAdd = 3;
        }
        /**
         * preset=0: default settings (2k pts etc.), not enforcing real-time execution
         * preset=1: default settings (2k pts etc.), enforcing 1x real-time execution
         * preset=2: fast settings (800 pts etc.), not enforcing real-time execution. WARNING: overwrites image resolution with 424 x 320.
         * preset=3: fast settings (800 pts etc.), enforcing 5x real-time execution. WARNING: overwrites image resolution with 424 x 320.
         * 0,2 效果正常，2会更好点；1,3直接飘走
         */
        settingsDefault(2);

        setting_logStuff = false;
        printf("DISABLE LOGGING!\n");

//        disableAllDisplay = true;
//        printf("NO GUI!\n");

        setting_debugout_runquiet = true;
        printf("QUIET MODE, Disable most console output!\n");
    }

//    FLAGS_colorlogtostderr = true;//    错误等级有颜色区分(Glog)
    setting_maxAffineWeight = 0.1;  // don't use affine brightness weight in Euroc!

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
