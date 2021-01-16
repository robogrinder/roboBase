//
// Created by Jason Li on 1/15/21.
//

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

#include <fstream>
#include <opencv2/dnn.hpp>

#include "../autoAim.h"

using namespace std;
using namespace cv;


TEST (SquareTest /*test suite name*/, PosZeroNeg /*test name*/) {
    EXPECT_EQ (9.0, (3.0*2.0)); // fail, test continues
    ASSERT_EQ (0.0, (0.0));     // success
    ASSERT_EQ (9, (3)*(-3.0));  // fail, test interrupts
    ASSERT_EQ (-9, (-3)*(-3.0));// not executed due to the previous assert
}


/*
int main() {
    String videoPath = "./infantry1.mp4";
    VideoCapture capture(videoPath);
    Mat frame;
    if (!capture.isOpened()) {
        perror("video not opened correctly");
        return -1;
    }
    capture.set(CAP_PROP_POS_FRAMES, START_FRAME);
    while (true) {
        capture >> frame;
        if (frame.empty())
            break;
        imshow("image", frame);
    }
    return 0;
}


*/