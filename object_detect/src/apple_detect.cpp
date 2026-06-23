#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <fstream>

struct Detection
{
    int class_id{0};
    std::string className{};
    float confidence{0.0};
    cv::Scalar color{};
    cv::Rect box{};
};

class AppleDetectNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_image_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr apple_pose_publisher_;

    cv::dnn::Net yolo_net_;
    std::vector<std::string> class_names_;

    float modelConfidenceThreshold_{0.55};
    float modelScoreThreshold_{0.55};
    float modelNMSThreshold_{0.7};
    cv::Size modelShape_{640, 640};
    bool letterBoxForSquare_{true};

    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg_;
    cv::Mat latest_color_image_;
    cv::Mat latest_depth_image_;
    bool got_camera_info_{false};
    bool got_color_image_{false};
    bool got_depth_image_{false};

    static constexpr int APPLE_CLASS_ID = 47;

public:
    AppleDetectNode() : Node("apple_detect")
    {
        load_class_names();
        load_yolo_model();

        camera_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/color/camera_info", 10,
            std::bind(&AppleDetectNode::camera_info_callback, this, std::placeholders::_1)
        );

        color_image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10,
            std::bind(&AppleDetectNode::color_image_callback, this, std::placeholders::_1)
        );

        depth_image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 10,
            std::bind(&AppleDetectNode::depth_image_callback, this, std::placeholders::_1)
        );

        annotated_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/apple_detect/image_out", 10
        );

        apple_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/apple_pose", 10
        );

        RCLCPP_INFO(this->get_logger(), "Apple detect node initialized");
    }

    void load_class_names()
    {
        class_names_ = {
            "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
            "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
            "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
            "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
            "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
            "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
            "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
            "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
            "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
            "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
        };
    }

    void load_yolo_model()
    {
        std::string model_path = "/usr/models/yolov8s.onnx";

        std::ifstream file(model_path);
        if (!file.good()) {
            RCLCPP_ERROR(this->get_logger(), "YOLOv8 model file not found: %s", model_path.c_str());
            RCLCPP_INFO(this->get_logger(), "Please ensure the YOLOv8 model is in %s", model_path.c_str());
            return;
        }
        file.close();

        try {
            yolo_net_ = cv::dnn::readNetFromONNX(model_path);
            yolo_net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            yolo_net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

            if (yolo_net_.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load YOLOv8 model: empty network");
            } else {
                RCLCPP_INFO(this->get_logger(), "YOLOv8 model loaded successfully: %s", model_path.c_str());
            }
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load YOLOv8 model: %s", e.what());
        }
    }

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        camera_info_msg_ = msg;
        got_camera_info_ = true;
    }

    void color_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            latest_color_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
            got_color_image_ = true;

            if (got_camera_info_ && got_depth_image_) {
                process_image(msg->header);
            }
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error converting color image: %s", e.what());
        }
    }

    void depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            latest_depth_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
            got_depth_image_ = true;

            if (got_camera_info_ && got_color_image_) {
                process_image(msg->header);
            }
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error converting depth image: %s", e.what());
        }
    }

    void process_image(const std_msgs::msg::Header& header)
    {
        if (yolo_net_.empty() || latest_color_image_.empty() || latest_depth_image_.empty()) {
            return;
        }

        std::vector<Detection> detections = run_inference(latest_color_image_);

        cv::Mat annotated_frame = latest_color_image_.clone();
        draw_detections(annotated_frame, detections);

        publish_annotated_image(annotated_frame, header);

        for (const auto& detection : detections) {
            calculate_and_publish_position(detection, header);
        }
    }

    std::vector<Detection> run_inference(const cv::Mat& input)
    {
        std::vector<Detection> detections;

        if (yolo_net_.empty()) {
            return detections;
        }

        try {
            cv::Mat modelInput = input;
            int pad_x = 0, pad_y = 0;
            float scale = 1.0;

            if (letterBoxForSquare_ && modelShape_.width == modelShape_.height) {
                modelInput = format_to_square(modelInput, &pad_x, &pad_y, &scale);
            }

            cv::Mat blob;
            cv::dnn::blobFromImage(modelInput, blob, 1.0/255.0, modelShape_, cv::Scalar(), true, false);
            yolo_net_.setInput(blob);

            std::vector<cv::Mat> outputs;
            yolo_net_.forward(outputs, yolo_net_.getUnconnectedOutLayersNames());

            if (outputs.empty()) {
                return detections;
            }

            int rows = outputs[0].size[1];
            int dimensions = outputs[0].size[2];

            if (dimensions > rows) {
                rows = outputs[0].size[2];
                dimensions = outputs[0].size[1];
                outputs[0] = outputs[0].reshape(1, dimensions);
                cv::transpose(outputs[0], outputs[0]);
            }

            float* data = (float*)outputs[0].data;

            std::vector<int> class_ids;
            std::vector<float> confidences;
            std::vector<cv::Rect> boxes;

            for (int i = 0; i < rows; ++i) {
                float* classes_scores = data + 4;
                cv::Mat scores(1, class_names_.size(), CV_32FC1, classes_scores);

                for (size_t j = 0; j < class_names_.size(); ++j) {
                    scores.at<float>(0, j) = 1.0f / (1.0f + exp(-scores.at<float>(0, j)));
                }

                cv::Point class_id;
                double maxClassScore;
                minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);

                if (maxClassScore > modelScoreThreshold_) {
                    confidences.push_back(static_cast<float>(maxClassScore));
                    class_ids.push_back(class_id.x);

                    float x = data[0];
                    float y = data[1];
                    float w = data[2];
                    float h = data[3];

                    int left = int((x - 0.5 * w - pad_x) / scale);
                    int top = int((y - 0.5 * h - pad_y) / scale);
                    int width = int(w / scale);
                    int height = int(h / scale);

                    boxes.push_back(cv::Rect(left, top, width, height));
                }
                data += dimensions;
            }

            std::vector<int> nms_result;
            cv::dnn::NMSBoxes(boxes, confidences, modelScoreThreshold_, modelNMSThreshold_, nms_result);

            for (size_t i = 0; i < nms_result.size(); ++i) {
                int idx = nms_result[i];

                if (class_ids[idx] != APPLE_CLASS_ID) {
                    continue;
                }

                Detection result;
                result.class_id = class_ids[idx];
                result.confidence = confidences[idx];
                result.className = class_names_[result.class_id];
                result.box = boxes[idx];

                detections.push_back(result);
            }
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in inference: %s", e.what());
        }

        return detections;
    }

    cv::Mat format_to_square(const cv::Mat& source, int* pad_x, int* pad_y, float* scale)
    {
        int col = source.cols;
        int row = source.rows;
        int m_inputWidth = modelShape_.width;
        int m_inputHeight = modelShape_.height;

        *scale = std::min(m_inputWidth / (float)col, m_inputHeight / (float)row);
        int resized_w = int(col * *scale);
        int resized_h = int(row * *scale);
        *pad_x = (m_inputWidth - resized_w) / 2;
        *pad_y = (m_inputHeight - resized_h) / 2;

        cv::Mat resized;
        cv::resize(source, resized, cv::Size(resized_w, resized_h));
        cv::Mat result = cv::Mat::zeros(m_inputHeight, m_inputWidth, source.type());
        resized.copyTo(result(cv::Rect(*pad_x, *pad_y, resized_w, resized_h)));
        return result;
    }

    void draw_detections(cv::Mat& frame, const std::vector<Detection>& detections)
    {
        cv::Scalar apple_color(0, 255, 0);

        for (const auto& detection : detections) {
            cv::Rect box = detection.box;
            cv::Scalar color = apple_color;

            box.x = std::max(0, box.x);
            box.y = std::max(0, box.y);
            box.width = std::min(frame.cols - box.x, box.width);
            box.height = std::min(frame.rows - box.y, box.height);

            cv::rectangle(frame, box, color, 2);

            std::string classString = detection.className + ' ' + std::to_string(detection.confidence).substr(0, 4);
            cv::Size textSize = cv::getTextSize(classString, cv::FONT_HERSHEY_DUPLEX, 0.6, 1, 0);
            cv::Rect textBox(box.x, box.y - textSize.height - 10, textSize.width + 10, textSize.height + 10);

            if (textBox.y < 0) {
                textBox.y = box.y + box.height;
            }
            if (textBox.x + textBox.width > frame.cols) {
                textBox.x = frame.cols - textBox.width;
            }

            cv::rectangle(frame, textBox, color, cv::FILLED);
            cv::putText(frame, classString, cv::Point(box.x + 5, textBox.y + textSize.height + 2),
                       cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(0, 0, 0), 1, 0);
        }
    }

    void calculate_and_publish_position(const Detection& detection, const std_msgs::msg::Header& header)
    {
        if (!camera_info_msg_ || latest_depth_image_.empty()) {
            return;
        }

        float fx = camera_info_msg_->k[0];
        float fy = camera_info_msg_->k[4];
        float cx = camera_info_msg_->k[2];
        float cy = camera_info_msg_->k[5];

        int box_x = std::max(0, detection.box.x);
        int box_y = std::max(0, detection.box.y);
        int box_width = std::min(latest_depth_image_.cols - box_x, detection.box.width);
        int box_height = std::min(latest_depth_image_.rows - box_y, detection.box.height);

        if (box_width <= 0 || box_height <= 0) {
            return;
        }

        std::vector<float> depth_values;
        for (int y = box_y; y < box_y + box_height; ++y) {
            for (int x = box_x; x < box_x + box_width; ++x) {
                uint16_t depth_mm = latest_depth_image_.at<uint16_t>(y, x);
                if (depth_mm > 0 && !std::isnan(static_cast<float>(depth_mm))) {
                    depth_values.push_back(static_cast<float>(depth_mm));
                }
            }
        }

        if (depth_values.empty()) {
            RCLCPP_WARN(this->get_logger(), "No valid depth values in bounding box");
            return;
        }

        std::sort(depth_values.begin(), depth_values.end());
        size_t median_start = depth_values.size() / 4;
        size_t median_end = depth_values.size() * 3 / 4;
        if (median_start >= median_end) {
            median_start = 0;
            median_end = depth_values.size();
        }

        float depth_sum = 0.0f;
        int count = 0;
        for (size_t i = median_start; i < median_end; ++i) {
            depth_sum += depth_values[i];
            count++;
        }

        if (count == 0) {
            RCLCPP_WARN(this->get_logger(), "No valid depth values after median filtering");
            return;
        }

        float depth_mm = depth_sum / count;
        float depth = depth_mm / 1000.0f;

        int center_x = detection.box.x + detection.box.width / 2;
        int center_y = detection.box.y + detection.box.height / 2;

        float x = (center_x - cx) * depth / fx;
        float y = (center_y - cy) * depth / fy;
        float z = depth;

        RCLCPP_INFO(this->get_logger(), "Detected %s at camera frame position: X=%.4f, Y=%.4f, Z=%.4f (meters)",
                   detection.className.c_str(), x, y, z);

        geometry_msgs::msg::PointStamped pose_msg;
        pose_msg.header = header;
        pose_msg.header.frame_id = "camera_color_optical_frame";
        pose_msg.point.x = x;
        pose_msg.point.y = y;
        pose_msg.point.z = z;

        apple_pose_publisher_->publish(pose_msg);
    }

    void publish_annotated_image(const cv::Mat& frame, const std_msgs::msg::Header& header)
    {
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        annotated_image_publisher_->publish(*msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AppleDetectNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}