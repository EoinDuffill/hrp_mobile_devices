#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

static const std::string IMAGE_PATH = "/home/eoin/ROS/catkin_ws/src/figure.jpg";
static const std::string TOPIC_NAME = "camera/image";

int publishImage(std::string filepath)
{
    Mat image;
    

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise(TOPIC_NAME, 1);

    ros::Rate loop_rate(0.5);

    while (nh.ok()) {

		//Read image 
		image = imread(filepath, CV_LOAD_IMAGE_COLOR);   // Read the file
		std::cout << "Path " << filepath << std::endl;
		if(!image.data)                              // Check for invalid input
		{
		    std::cout << "Could not open or find the image" << std::endl ;
		    return -1;
		}
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_transport_publisher");
    publishImage(IMAGE_PATH);
    return 0;
}
