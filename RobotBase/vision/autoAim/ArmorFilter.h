//
// Created by jsz on 12/28/19.
//

#ifndef ROBOTBASE_ARMORFILTER_H
#define ROBOTBASE_ARMORFILTER_H

#include <opencv2/opencv.hpp>

/**
 * the LED_bar is the abstraction object of a LED bar in the ArmorFilter plate.
 */
class LED_bar {
public:
    LED_bar() : matched(false) {}

    /**
     *  This is the the constructor of Class
     * @param R the info of LED bar
     */
    LED_bar(const cv::RotatedRect &R) {
        rect.angle = R.angle;
        rect.center = R.center;
        rect.size = R.size;
        matched = false;
    }

    cv::RotatedRect rect;
    bool matched;
    size_t match_index;
    float match_factor;
};


/**
 * This is the ArmorFilter class, which is the object of ArmorFilter plate on the robot
 *  which is formed by two LED_bar object.
 */
class ArmorFilter {
public:
    ArmorFilter();
    /**
     *  the constructor
     * @param left  left LED bar
     * @param right Right LED bar
     */
    ArmorFilter(const LED_bar &left, const LED_bar &right);

    void draw_rect(cv::Mat &img, cv::Point2f roi_offset_point) const;

    void draw_spot(cv::Mat &img, cv::Point2f roi_offset_point) const;

    /**
     * get the average intensity
     * Intensity in the Computer vision means how bright (mean pixel intensity) the image appears
     * @param img the image passed in
     * @return the average value of image intensity
     */
    int get_average_intensity(const cv::Mat &img);

    /**
     * iterate the LEDs vector to match every possible lED_bars
     * @param LEDs the vector of LED_bar
     * @param i the index of LED_bar
     * @param j the index of LED_bar
     */
    void max_match(std::vector<LED_bar> &LEDs, size_t i, size_t j);

    /**
     * If the two led bar can form a ArmorFilter or not.
     * @return True: suitable to form a ArmorFilter
     *         False: not suitable to form a ArmorFilter
     **/
    bool is_suitable_size();

    LED_bar led_bars[2];
    float error_angle;
    cv::Point2i center;
    cv::Rect2i rect;
    int average_intensity;

};

#endif //ROBOTBASE_ARMORFILTER_H
