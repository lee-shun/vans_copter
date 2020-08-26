#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW="Image window";

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    int frame;

    public:
    ImageConverter():it_(nh_)
    {
        image_sub_=it_.subscribe("/copter/cam/copterCam", 1, &ImageConverter::imageCb, this);
        image_pub_=it_.advertise("/processed_image", 1);
        this->frame=0;

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter()
    {
        // cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr=cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // if(cv_ptr->image.rows>60&&cv_ptr->image.cols>60)
        //     cv::circle(cv_ptr->image, cv::Point(50,50), 10, CV_RGB(255,0,0));
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::imwrite("outputImg.jpg", cv_ptr->image);
        this->frame++;
        // ROS_INFO("frame: %d", this->frame);
        cv::waitKey(3);

        image_pub_.publish(cv_ptr->toImageMsg());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}