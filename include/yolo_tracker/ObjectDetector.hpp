#ifndef _OBJECT_DETECTOR_HPP
#define _OBJECT_DETECTOR_HPP

#include <opencv2/dnn.hpp>
#include <string>

struct ProcessingResult {
    ProcessingResult(): rectangle(cv::Rect()), class_id(0), confidence(0) {}
    ProcessingResult(cv::Rect rect, int id, double conf) : rectangle(rect), class_id(id), confidence(conf) {}

    cv::Rect rectangle;
    int class_id;
    double confidence;
};

class ObjectDetector {
private:
    std::string object_to_detect;
    int in_w;
    int in_h;
    float detection_threshold;
    float nms_threshold;
    bool prev_detected;
    int prev_detected_center_x, prev_detected_center_y;
    
    cv::dnn::Net net;
    std::vector<std::string> classes;
    cv::Mat blob;
    std::vector<cv::Mat> outs;
    std::vector<std::string> detection_names;

    void _detect_objects(cv::Mat &frame, std::vector<ProcessingResult> &res);
    double _distance_to_prev(cv::Rect &rect);

public:
    ObjectDetector(const std::string& object_to_detect, const std::string &model_config_file, const std::string &model_weights_file,
                   const std::string &names_file, float threshold,
                   float nms_threshold, int model_input_width = 320, int model_input_height = 320);

    bool detect_object(cv::Mat &frame, ProcessingResult &result);
    
};

#endif
