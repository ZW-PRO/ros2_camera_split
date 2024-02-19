#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

class CameraSplitter : public rclcpp::Node
{
public:
    CameraSplitter() : Node("camera_splitter")
    {
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/usb_cam/image_raw", rclcpp::QoS(rclcpp::KeepLast(2000)), std::bind(&CameraSplitter::imageCallback, this, std::placeholders::_1));
        std::cout << "11111" << std::endl;
        image_pub_left_ = create_publisher<sensor_msgs::msg::Image>("/left_cam/image_raw", 1);
        image_pub_right_ = create_publisher<sensor_msgs::msg::Image>("/right_cam/image_raw", 1);
        cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);

        // 读取参数服务器参数，得到左右相机参数文件的位置
        std::string left_cal_file = declare_parameter<std::string>("left_cam_file", "");
        std::string right_cal_file = declare_parameter<std::string>("right_cam_file", "");
        if (!left_cal_file.empty())
        {
            if (cinfo_->validateURL(left_cal_file))
            {
                RCLCPP_INFO(get_logger(), "加载左相机信息文件：%s", left_cal_file.c_str());
                cinfo_->loadCameraInfo(left_cal_file);
                ci_left_ = std::make_shared<sensor_msgs::msg::CameraInfo>(cinfo_->getCameraInfo());
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "无法加载左相机信息文件：%s", left_cal_file.c_str());
                rclcpp::shutdown();
            }
        }
        else
        {
            RCLCPP_INFO(get_logger(), "未指定左相机信息文件。");
            ci_left_ = std::make_shared<sensor_msgs::msg::CameraInfo>();
        }
        if (!right_cal_file.empty())
        {
            if (cinfo_->validateURL(right_cal_file))
            {
                RCLCPP_INFO(get_logger(), "加载右相机信息文件：%s", right_cal_file.c_str());
                cinfo_->loadCameraInfo(right_cal_file);
                ci_right_ = std::make_shared<sensor_msgs::msg::CameraInfo>(cinfo_->getCameraInfo());
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "无法加载右相机信息文件：%s", right_cal_file.c_str());
                rclcpp::shutdown();
            }
        }
        else
        {
            RCLCPP_INFO(get_logger(), "未指定右相机信息文件。");
            ci_right_ = std::make_shared<sensor_msgs::msg::CameraInfo>();
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        namespace enc = sensor_msgs::image_encodings;
        
        cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
        std::cout << "444444" << std::endl;
        
        //std::cout << cv_ptr->image.cols << std::endl;
        //std::cout << cv_ptr->image.rows << std::endl;
        // 截取ROI(Region Of Interest)，即左右图像，会将原图像数据拷贝出来。
        leftImgROI_ = cv_ptr->image(cv::Rect(0, 0, cv_ptr->image.cols / 2, cv_ptr->image.rows));
        rightImgROI_ = cv_ptr->image(cv::Rect(cv_ptr->image.cols / 2, 0, cv_ptr->image.cols / 2, cv_ptr->image.rows));
        std::cout << "Left ROI dimensions: " << leftImgROI_.rows << " x " << leftImgROI_.cols << std::endl;
	std::cout << "Right ROI dimensions: " << rightImgROI_.rows << " x " << rightImgROI_.cols << std::endl;

        // 创建两个CvImage，用于存放原始图像的左右部分。CvImage创建时是对Mat进行引用的，不会进行数据拷贝
        leftImgPtr_ = std::make_shared<cv_bridge::CvImage>(cv_ptr->header, enc::BGR8, leftImgROI_);
        rightImgPtr_ = std::make_shared<cv_bridge::CvImage>(cv_ptr->header, enc::BGR8, rightImgROI_);

        // 发布到/left_cam/image_raw和/right_cam/image_raw
        ci_left_->header = cv_ptr->header; // 很重要，不然会提示不同步导致无法去畸变
        ci_right_->header = cv_ptr->header;
        auto leftPtr = leftImgPtr_->toImageMsg();
        auto rightPtr = rightImgPtr_->toImageMsg();
        leftPtr->header = msg->header; // 很重要，不然输出的图像没有时间戳
        rightPtr->header = msg->header;
        image_pub_left_->publish(*leftPtr);
        image_pub_right_->publish(*rightPtr);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_left_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_right_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> ci_left_;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> ci_right_;

    cv::Mat leftImgROI_;
    cv::Mat rightImgROI_;
    cv_bridge::CvImagePtr leftImgPtr_ = nullptr;
    cv_bridge::CvImagePtr rightImgPtr_ = nullptr;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraSplitter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

