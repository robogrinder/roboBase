//
// Created by jsz on 12/19/19.
//


#include "autoAim.h"

using namespace cv;
using namespace std;


/**
 * If the ArmorDetector found a armor in previous 15 frames:
 *      <br>then roi will be a slightly bigger rectangle based on most recent roi
 * Else:
 *      <br>then roi will be entire picture
 * @param img image to look for
 * @return region of interst
 */
cv::Rect ArmorDetector::GetRoi(const cv::Mat &img) {
    Size img_size = img.size();
    // TODO: why use rect_tmp when it's the same as last_target? Or using &rect_tmp is better?
    Rect rect_tmp = last_target_;
    Rect rect_roi;
    if (rect_tmp.x == 0 || rect_tmp.y == 0
        || rect_tmp.width == 0 || rect_tmp.height == 0
        || lost_count >= 15) {
        last_target_ = Rect(0, 0, img_size.width, img_size.height);
        rect_roi = Rect(0, 0, img_size.width, img_size.height);
        return rect_roi;
    } else {
        // TODO: previous if already means lost_count < 15, so these if/else are unnecessary?
        float scale = 2;
        if (lost_count <= 35)
            scale = 2.3;
        else if (lost_count <= 60)
            scale = 3;
        else if (lost_count <= 100)
            scale = 3.5;

        int w = int(rect_tmp.width * scale);
        int h = int(rect_tmp.height * scale);
        int x = int(rect_tmp.x - (w - rect_tmp.width) * 0.5f);
        int y = int(rect_tmp.y - (h - rect_tmp.height) * 0.5f);

        rect_roi = Rect(x, y, w, h);

        if (!makeRectSafe(rect_roi, img_size)) {
            rect_roi = Rect(0, 0, img_size.width, img_size.height);
        }
    }
    return rect_roi;
}

/**
 * try to find armor within roi inside img.
 * @param img original image
 * @param roi only search this region inside img
 * @return true if a valid armor is detected; false otherwise
 */
bool ArmorDetector::detectArmor(cv::Mat &img, const cv::Rect &roi) {
    Mat roi_image = img(roi);
    Point2f offset_roi_point(roi.x, roi.y);
    vector<Mat> BGR_channels;
    vector<LED_bar> LED_bars;
    bool found_flag = false;
    Mat bin_gray_img, bin_color_img, gray, debug_img, color_result_img;;
    debug_img = img.clone();

    cvtColor(roi_image, gray, COLOR_BGR2GRAY);
    split(roi_image, BGR_channels);
    if (color_ == 0) // opposite red
    {
        subtract(BGR_channels[2], BGR_channels[1], color_result_img);
    } else {
        subtract(BGR_channels[0], BGR_channels[2], color_result_img);
    }
    threshold(gray, bin_gray_img, gray_th_, 255, THRESH_BINARY);
    threshold(color_result_img, bin_color_img, color_th_, 255, THRESH_BINARY);

#if SHOW_BINART
    imshow("bin_gray_img", bin_gray_img);
    imshow("bin_color_img", bin_color_img);
#endif
    vector<vector<Point> > contours_color;
    vector<vector<Point> > contours_gray;
    findContours(bin_color_img, contours_color, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    findContours(bin_gray_img, contours_gray, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    //printf("%zu\n",contours_color.size());
    //printf("%zu\n",contours_gray.size());
    /**
     * still need to test
     */
    if (contours_gray.size() < 2 || contours_color.size() < 2 || contours_gray.size() > 10 ||
        contours_color.size() > 10) {
        //imshow("debug_img", debug_img);
        //waitKey(1);
        return found_flag;
    }

    for (unsigned int i = 0; i < contours_gray.size(); i++) {
        double area = contourArea(contours_gray[i]);
        if (area > 1e5) {
            continue;
        }
        for (unsigned int j = 0; j < contours_color.size(); j++) {
            if (pointPolygonTest(contours_color[j], contours_gray[i][0], false) >= 0.0) {
                double length = arcLength(contours_gray[i], true); // LED circumference
                if (20 < length && length < 4000) {
                    RotatedRect RRect = fitEllipse(contours_gray[i]);
                    auto light_aspect_ratio =
                            std::max(RRect.size.width, RRect.size.height) /
                            std::min(RRect.size.width, RRect.size.height);
                    // TODO: why not 180f?
                    if (RRect.angle > 90.0f)
                        RRect.angle = RRect.angle - 180.0f;

                    if (fabs(RRect.angle) < 30) {
#if SHOW_LIGHT_CONTOURS
                        char temp[20];
                        sprintf(temp, "%0.2f", light_aspect_ratio);
                        putText(debug_img, temp, RRect.center + Point2f(0, -40) + offset_roi_point,
                                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
                        Point2f rect_point[4];
                        RRect.points(rect_point);
                        for (int i = 0; i < 4; i++) {
                            line(debug_img, rect_point[i] + offset_roi_point,
                                 rect_point[(i + 1) % 4] + offset_roi_point,
                                 Scalar(255, 0, 255), 1);
                        }

                        char temp1[20];
                        sprintf(temp1, "%0.2f", RRect.angle);
                        putText(debug_img, temp1, RRect.center + Point2f(0, -10) + offset_roi_point,
                                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
#endif
                        LED_bar r(RRect);
                        LED_bars.emplace_back(r);

                    }
                }
                break;
            }
        }
    }
    //==========================================possible Armor=========================================
    /*
     * try out all combinations of LED_bars. If two LEDs are almost parallel to each other
     * then they are paired together by modifying the LED fields (matched, match_index, etc.)
     */
    for (size_t i = 0; i < LED_bars.size(); i++) {
        for (size_t j = i + 1; j < LED_bars.size(); j++) {
            Armor temp_armor(LED_bars.at(i), LED_bars.at(j));
            if (temp_armor.error_angle < 7.0f && temp_armor.is_suitable_size() &&
                temp_armor.get_average_intensity(gray) < 70) {
                // modify fields to match LEDs at index i and j
                temp_armor.max_match(LED_bars, i, j);
            }
        }
    }
    //====================================find final armors============================================
    vector<Armor> final_armor_list;

    for (size_t i = 0; i < LED_bars.size(); i++) {
        if (LED_bars.at(i).matched) {
            LED_bars.at(LED_bars.at(i).match_index).matched = false; //clear another matching flag
            Armor arm_tmp(LED_bars.at(i), LED_bars.at(LED_bars.at(i).match_index));
            //arm_tmp.draw_rect(debug_img, offset_roi_point);
            final_armor_list.push_back(arm_tmp);
        }
    }

    // get the closest armor inside final_armor_list compared to center of roi
    // result is stored in "target" variable
    float dist = 1e8;

    Armor target;
    // TODO: what is roi?
    Point2f roi_center(roi.width / 2, roi.height / 2);
    float dx, dy;
    for (auto &i : final_armor_list) {
#if FAST_DISTANCE
        dx = fabs(final_armor_list.at(i).center.x - roi_center.x);
        dy = fabs(final_armor_list.at(i).center.y - roi_center.y);
#else
        dx = pow((i.center.x - roi_center.x), 2.0f);
        dy = pow((i.center.y - roi_center.y), 2.0f);
#endif
        if (dx + dy < dist) {
            target = i;
            dist = dx + dy;
        }
#if SHOW_FINAL_ARMOR
        i.draw_rect(debug_img, offset_roi_point);

#endif
        found_flag = true;
    }
#if SHOW_ROI
    rectangle(debug_img, roi, Scalar(255, 0, 255), 1);
#endif
    // if final_armor_list is not empty (found a final candidate):
    if (found_flag) {
#if SHOW_DRAW_SPOT
        target.draw_spot(debug_img, offset_roi_point);
#endif

        Point2f point_tmp[4];
        Point2f point_2d[4];

        // classify the left and right LED_bar, and abstrct four corner points
        RotatedRect R, L;
        if (target.led_bars[0].rect.center.x > target.led_bars[1].rect.center.x) {
            R = target.led_bars[0].rect;
            L = target.led_bars[1].rect;
        } else {
            R = target.led_bars[1].rect;
            L = target.led_bars[0].rect;
        }
        // find armor rectangle that's between two LEDs and stores inside "point_2d"
        L.points(point_tmp);
        point_2d[0] = point_tmp[1];
        point_2d[3] = point_tmp[0];
        R.points(point_tmp);
        point_2d[1] = point_tmp[2];
        point_2d[2] = point_tmp[3];
        vector<Point2f> points_roi_tmp;
        final_armor_2Dpoints.clear();
        for (int i = 0; i < 4; i++) {
            // TODO: aren't points_roi_tmp and final_armor_2Dpoints the same?
            points_roi_tmp.push_back(point_2d[i] + offset_roi_point);
            final_armor_2Dpoints.push_back(point_2d[i] + offset_roi_point);
            circle(debug_img, final_armor_2Dpoints.at(i), 5, Scalar(255, 255, 255), -1);
            circle(debug_img, final_armor_2Dpoints.at(i), 3, Scalar(i * 50, i * 50, 255), -1);
        }

        float armor_h = target.rect.height;
        float armor_w = target.rect.width;
        is_small_ = armor_w / armor_h < 3.3f;

        //get the new target
        last_target_ = boundingRect(points_roi_tmp);
#if SHOW_LAST_TARGET
        rectangle(debug_img, last_target_, Scalar(255, 255, 255), 1);
#endif
        lost_count = 0;
    } else {
        lost_count++;
    }
    detect_count++;
    imshow("debug_img", debug_img);
    waitKey(1);
    return found_flag;
}

/**
 * starting point for ArmorDetector; called by the thread management
 * @param color_img image captured by camera
 * @param other_param
 * @param sp
 * @return
 */
int ArmorDetector::armorTask(cv::Mat &color_img, OtherParam other_param, serial_port sp) {
    // TODO: what is level_ and where is it used?
    color_ = other_param.color;
    mode_ = other_param.mode;
    level_ = other_param.level;

#if ROI_ENABLE
    Rect roi = GetRoi(color_img);
#else
    Size img_size = color_img.size();
    Rect roi = Rect(0, 0, img_size.width, img_size.height);
#endif
    Point3f target_3d = {0, 0, 0};
    Mat rvec; // not used
    Mat tvec;
    // TODO: why are they initialized, but then calculated again here? Use zero instead?
    OFFSET_YAW = (OFFSET_INT_YAW - 1800);
    OFFSET_PITCH = (OFFSET_INT_PITCH - 1800);
    if (detectArmor(color_img, roi)) {
        printf("detected\n");
        if (is_small_) {
            solvePnP(small_real_armor_points, final_armor_2Dpoints, cameraMatrix,
                     distCoeffs, rvec, tvec, false, SOLVEPNP_ITERATIVE);
        } else {
            solvePnP(big_real_armor_points, final_armor_2Dpoints, cameraMatrix,
                     distCoeffs, rvec, tvec, false, SOLVEPNP_ITERATIVE);
        }

        target_3d = cv::Point3f(tvec);
        printf("x:%f y:%f z:%f\n", target_3d.x, target_3d.y, target_3d.z);

        // TODO: change this -80 offset to a variable based on physical characteristics of the robot?
        // height needs to - 80 because difference between camera and turret
        int pitch = int((atan2(target_3d.y - 80, target_3d.z) + (float) (OFFSET_PITCH * CV_PI / 1800)) * 0.6 * 10000);
        int yaw = int((-atan2(target_3d.x, target_3d.z) + (float) (OFFSET_YAW * CV_PI / 1800)) * 0.4 * 10000);
/*
        //int yaw = -15000;
        //printf("yaw: %d, pitch: %d\n", yaw, pitch);
        //pitch_vector.push_back((float)pitch/10000);
//        yaw_array[yaw_array_count] = yaw;
//        yaw_array_count++;
//        yaw_array_size++;
//        if (yaw_array_count > 2)
//            yaw_array_count = 0;
//        if (yaw_array_size > 3)
//            yaw_array_size = 3;
//        float total = 0;
//        for (int i = 0; i < yaw_array_size; i++) {
//            total += yaw_array[i];
//            //printf("%d ", yaw_array[i]);
//        }
//        total = (float)(total / (yaw_array_size));
//        printf("\npredit speed: %f\n", total);
//        yaw += total * 1.4f;
*/
        struct serial_gimbal_data data;
        data.size = 6;
        data.rawData[0] = data.head;
        data.rawData[1] = data.id;
        data.rawData[2] = pitch;
        data.rawData[3] = pitch >> 8;
        data.rawData[4] = yaw;
        data.rawData[5] = yaw >> 8;

        sp.send_data(data);
    } else {
        //printf("not detected\n");
        int pitch = 0;
        //int pitch = 15000;
        int yaw = 0;
        struct serial_gimbal_data data;
        data.size = 6;
        data.rawData[0] = data.head;
        data.rawData[1] = data.id;
        data.rawData[2] = pitch;
        data.rawData[3] = pitch >> 8;
        data.rawData[4] = yaw;
        data.rawData[5] = yaw >> 8;

        sp.send_data(data);
    }

}


