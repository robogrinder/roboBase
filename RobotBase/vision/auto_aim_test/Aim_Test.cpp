//
// Created by Jason Li on 1/15/21.
//

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

#include <string>
#include <iostream>
#include <filesystem>
#include <unistd.h>
#include "../autoAim/autoAim.h"
#include "../../Robogrinder_SDK/serial_port.h"

using namespace std;
using namespace cv;

#include "ArmorFilter.h"
int main() {

    String videoPath = "../../../../RobotBase/vision/auto_aim_test/vid1.mp4";
    VideoCapture capture(videoPath);
    Mat frame;
    if (!capture.isOpened()) {
        perror("video not opened correctly");
        return -1;
    }
    // capture.set(CAP_PROP_POS_FRAMES, START_FRAME);
    int totalFrames = 0;
    int foundCount = 0;
    ArmorDetector detector;
    for(int i = 0;; i++) {
        capture.read(frame);
        capture >> frame;
        if (frame.empty())
            break;

        detector.armorTask(frame, OtherParam(), serial_port());

        vector<Point_<float>> finalArmor = detector.getFinalArmor();
        imshow("test", frame);

        if (finalArmor.size() > 0) {
            // cout << "image found\n\n\n!";
            foundCount++;
            //waitKey(200);
        }
        else {
            // cout << "not found\n";
            //waitKey(200);
        }
        totalFrames++;

    }
    cout << "totalFrames found: " << foundCount << ", totalFrames frames: " << totalFrames << endl;
    cout << "percentage of frames found: " << ((foundCount + 0.0) / totalFrames);
    return 0;
}


