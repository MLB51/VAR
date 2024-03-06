#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
using namespace std;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try { 
    // Resize the image
        cv::Mat originalImage;
        cv::Mat resizedImage;
        
        originalImage = cv_bridge::toCvShare(msg, "bgr8")->image;
        // la camara es tocha, se redimensiona para que no ocupe toda la pantalla y poder funcionar mejor
        cv::resize(originalImage, resizedImage, cv::Size(640, 480)); 

        cv::imshow("view", resizedImage);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv) {
    // inicializa el nodo
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    cv::namedWindow("view");
    cv::startWindowThread();
    // movidas para trabajar con opencv
    image_transport::ImageTransport it(nh);
    // se suscribe a la camara para coger las imgs en rgb
    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, imageCallback);
    
    // tambien se podria cambiar a depth pero requiere de debug porque son datos distintos
    // image_transport::Subscriber sub = it.subscribe("/camera/depth/image_raw", 1, imageCallback);
    ros::Rate rate(10.0); // 10 hz
    while(nh.ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    ros::shutdown();
    cv::destroyWindow("view");
}