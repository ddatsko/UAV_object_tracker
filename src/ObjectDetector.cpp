#include <fstream>
#include <iostream>

#include "yolo_tracker/ObjectDetector.hpp"

namespace dnn = cv::dnn;

ObjectDetector::ObjectDetector(const std::string &object_to_detect, const std::string &model_config_file, const std::string &model_weights_file,
                               const std::string &names_file, float threshold, float nms_threshold,
                               int model_input_width, int model_input_height)
        : object_to_detect(object_to_detect), in_w(model_input_width), in_h(model_input_height), detection_threshold(threshold),
          nms_threshold(nms_threshold) {
    prev_detected = false;
    
    // Read names file
    std::ifstream ifs(names_file.c_str());
    std::string line;
    while (getline(ifs, line))
        classes.push_back(line);

    net = dnn::readNetFromDarknet(model_config_file, model_weights_file);

    // May change this to GPU if OpenCV and DNN were compiled with CUDA
    net.setPreferableBackend(dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(dnn::DNN_TARGET_CPU);


    // Fill the mapping of detected index to object name (e.g. 0 to "person")
    std::vector<int> out_layers = net.getUnconnectedOutLayers();
    std::vector<std::string> layers_names = net.getLayerNames();
    detection_names.resize(out_layers.size());
    for (size_t i = 0; i < out_layers.size(); i++) {
        detection_names[i] = layers_names[out_layers[i] - 1];
    }
}

double ObjectDetector::_distance_to_prev(cv::Rect &rect) {
    double d_x = rect.x + rect.width / 2 - prev_detected_center_x;
    double d_y = rect.y + rect.height / 2 - prev_detected_center_y;
    return std::sqrt(d_x * d_x + d_y * d_y); 
}

bool ObjectDetector::detect_object(cv::Mat &frame, ProcessingResult &result) {
    std::vector<ProcessingResult> objects;
    _detect_objects(frame, objects);
    bool suitable_found = false;
    for (auto &object: objects) {
        // If obkect is of wrong class - skip it
        if (classes[object.class_id] != object_to_detect) {
            continue;
        }
        // If this is the first suitable item found
        if (!suitable_found || !prev_detected) {
            result = object;
        } else {
            // Choose the rectangle, closest to the previous detected object
            if (_distance_to_prev(result.rectangle) > _distance_to_prev(object.rectangle)) {
                result = object;
            }
        }
        suitable_found = true;
    }
    if (suitable_found) {
        prev_detected = true;
        prev_detected_center_x = result.rectangle.x + result.rectangle.width / 2;
        prev_detected_center_y = result.rectangle.y + result.rectangle.height / 2;
    } else {
        prev_detected = false;
    }
    return suitable_found;
}


void ObjectDetector::_detect_objects(cv::Mat &frame, std::vector<ProcessingResult> &res) {
    outs.clear();
    cv::Rect cropped_area = cv::Rect((frame.cols - in_w) / 2, (frame.rows - in_h) / 2, in_w, in_h);
    frame = frame(cropped_area);

    dnn::blobFromImage(frame, blob, 1 / 255.0, cv::Size(in_w, in_h), cv::Scalar(), true, false);

    net.setInput(blob);

    net.forward(outs, detection_names);

    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;

    // Select all the objects with high confidence
    for (size_t i = 0; i < outs.size(); i++) {
        float *data = (float *) outs[i].data;
        for (int j = 0; j < outs[i].rows; j++, data += outs[i].cols) {
            cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            cv::Point class_id_point;
            double confidence;
            minMaxLoc(scores, 0, &confidence, 0, &class_id_point);
            if (confidence > detection_threshold) {
                int width = (int) (data[2] * frame.cols);
                int height = (int) (data[3] * frame.rows);
                int left = (int) (data[0] * frame.cols) - width / 2;
                int top = (int) (data[1] * frame.rows) - height / 2;

                boxes.push_back(cv::Rect(left, top, width, height));
                confidences.push_back(confidence);
                class_ids.push_back(class_id_point.x);
            }
        }
    }

    // Remove overlapping boxes
    std::vector<int> indices;
    dnn::NMSBoxes(boxes, confidences, detection_threshold, nms_threshold, indices);
    for (size_t i = 0; i < indices.size(); ++i) {
        int idx = indices[i];
        res.push_back(ProcessingResult(boxes[idx], class_ids[idx], confidences[idx]));
    }
}
