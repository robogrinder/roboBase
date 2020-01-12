//
// Created by eric on 11/30/19.
//

#include "vision_main.h"

using namespace cv;

void *vision_main_function() {
    rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 60);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
    rs2::pipeline_profile pipelineProfile = pipe.start(cfg);

    pipelineProfile.get_device().query_sensors()[1].set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
    pipelineProfile.get_device().query_sensors()[1].set_option(RS2_OPTION_EXPOSURE, 7);
    pipelineProfile.get_device().query_sensors()[1].set_option(RS2_OPTION_GAIN, 0);

    pipelineProfile.get_device().query_sensors()[0].set_option(RS2_OPTION_LASER_POWER, 360);

    // Camera warmup - dropping several first frames to let auto-exposure stabilize


    ArmorDetector armorDetector;
    armorDetector.depth_intrinsics = pipelineProfile.get_stream(
            RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
//    armorDetector.color_intrinsics = pipelineProfile.get_stream(
//            RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
    //armorDetector.depth_scale = pipelineProfile.get_device().first<rs2::depth_sensor>().get_depth_scale();
//    armorDetector.depth_extrin_to_color = frames.get_depth_frame().get_profile().as<rs2::video_stream_profile>().get_extrinsics_to(
//            frames.get_color_frame().get_profile());
//    armorDetector.color_extrin_to_depth = frames.get_color_frame().get_profile().as<rs2::video_stream_profile>().get_extrinsics_to(
//            frames.get_depth_frame().get_profile());
    rs2::frameset frames;
    rs2::colorizer color_map;
    OtherParam otherParam;
    namedWindow("control", WINDOW_AUTOSIZE);
    createTrackbar("color_th", "control", &armorDetector.color_th_, 255);
    createTrackbar("gray_th", "control", &armorDetector.gray_th_, 255);
    while (1) {

        frames = pipe.wait_for_frames();
        rs2::frame color_frame = frames.get_color_frame();
//        rs2::depth_frame depth_frame = frames.get_depth_frame();
//
        Mat color(Size(640, 480), CV_8UC3, (void *) color_frame.get_data(), Mat::AUTO_STEP);
        //Mat depth(Size(640, 480), CV_8UC3, (void *) depth_frame.get_data(), Mat::AUTO_STEP);
        //imshow("depth", depth);
        otherParam.level = gimbal->getLevel();
        otherParam.id = gimbal->getId();
        if (otherParam.id == 13) // blue 3
        {
            otherParam.color = 1;
        } else {
            otherParam.color = 1;
        }
//        auto time0 = static_cast<double>(getTickCount());
        armorDetector.armorTask(color, frames, otherParam);
        //time0 = ((double) getTickCount() - time0) / getTickFrequency();
        //std::cout << "use time is " << time0 * 1000 << "ms" << std::endl;
        if (waitKey(1) == 'q') {

            break;
        }
    }
    pipe.stop();
    destroyAllWindows();
    return nullptr;
}

