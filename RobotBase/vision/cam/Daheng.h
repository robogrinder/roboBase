//
// Created by jsz on 2/15/20.
//

#ifndef ROBOTBASE_DAHENG_H
#define ROBOTBASE_DAHENG_H

#include <opencv2/opencv.hpp>
#include "GxIAPI.h"
#include "DxImageProc.h"

using namespace cv;
using namespace std;
/**
 * The 'Daheng' class is the object to obtain the image from camera
 * For more information, you can access the GxiAPI.h and SDK manual pdf in the project folder
 */
class Daheng {
public:
    /**
     * This is the constructor
     */
    Daheng();
    /**
     * This is the destructor
     */
    ~Daheng();
    /**
     * This is the initiation function to set up the camera parameters
     * @return
     * 0 means initiation failed
     */
    int init();
    /**
     * the 'getImage' function gets the one frame from the camera and store it into 'img'
     * @param img - the container of the frame
     */
    void getImage(Mat &img);
    // Todo: retuen the number of total frames gotten from camera
    uint64_t getFrameNumber();


private:

    GX_STATUS status;
    GX_DEV_HANDLE hDevice;
    GX_OPEN_PARAM stOpenParam;
    uint32_t nDeviceNum;
    GX_FRAME_DATA stFrameData;
    Mat src;
    uint64_t nFrameNum;
};


#endif //ROBOTBASE_DAHENG_H
