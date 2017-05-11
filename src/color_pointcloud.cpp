#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <Eigen/Dense>

ros::Publisher pub_pc;
image_transport::Publisher pub_camera;
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
Eigen::MatrixXd tmat;
Eigen::MatrixXd pmat;
Eigen::MatrixXd rmat;
cv::Mat rect_view;

static const std::string OPENCV_WINDOW = "Image window";

void camera_cb(const sensor_msgs::ImageConstPtr& msg)
{

  cv::Size imageSize = cv::Size(msg->width,msg->height);
  cv_bridge::CvImagePtr cv_cam = cv_bridge::toCvCopy(msg, "rgb8");

  rect_view = cv_cam->image;
  
  //cv::imshow(OPENCV_WINDOW,  rect_view);
  sensor_msgs::ImagePtr msg_send = cv_bridge::CvImage(std_msgs::Header(), "rgb8", rect_view).toImageMsg();

  pub_camera.publish(msg_send);
}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  //ROS_INFO("Got point cloud!");

  pcl::fromROSMsg(*input, *laserCloudIn);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colour_laser_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

  for(unsigned int i = 0; i < laserCloudIn->size(); i++)
  {
    float x3d = laserCloudIn->at(i).x;
    float y3d = laserCloudIn->at(i).y;
    float z3d = laserCloudIn->at(i).z;

    Eigen::MatrixXd point = Eigen::MatrixXd(4,1);
    Eigen::MatrixXd point_t = Eigen::MatrixXd(4,1);
    point << x3d, y3d, z3d, 1;

    point_t = pmat * rmat * tmat * point;

    pcl::PointXYZRGB p;
    p.x = x3d;
    p.y = y3d;
    p.z = z3d;
    p.r = 0;
    p.g = 0;
    p.b = 0;

    if (point_t(2,0) > 0) {

      int image_u = (int)(point_t(0,0)/(point_t(2,0)));
      int image_v = (int)(point_t(1,0)/(point_t(2,0)));
      
      if (0 <= image_u && image_u < rect_view.size().width &&
          0 <= image_v && image_v < rect_view.size().height) {
        cv::Vec3b color= rect_view.at<cv::Vec3b>(cv::Point(image_u, image_v));
        p.r = color[0];
        p.g = color[1];
        p.b = color[2];

      }

    }
    colour_laser_cloud->push_back(p);
    
  }

  sensor_msgs::PointCloud2 scan_color = sensor_msgs::PointCloud2();
  pcl::toROSMsg(*colour_laser_cloud, scan_color);
  scan_color.header.frame_id = "velo_link";
  pub_pc.publish (scan_color);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");


  tmat = Eigen::MatrixXd(4,4);
  tmat <<   7.533745e-03, -9.999714e-01, -6.166020e-04, -4.069766e-03,
            1.480249e-02,7.280733e-04, -9.998902e-01, -7.631618e-02,
             9.998621e-01, 7.523790e-03, 1.480755e-02, -2.717806e-01,
             0.0f, 0.0f, 0.0f, 1.0f;

  pmat = Eigen::MatrixXd(3,4);
  pmat << 7.215377e+02, 0.000000e+00, 6.095593e+02,
   0.000000e+00, 0.000000e+00, 7.215377e+02,
    1.728540e+02, 0.000000e+00, 0.000000e+00,
     0.000000e+00, 1.000000e+00, 0.000000e+00;

  rmat = Eigen::MatrixXd(4,4);
  rmat << 9.999239e-01, 9.837760e-03, -7.445048e-03,0,
   -9.869795e-03, 9.999421e-01, -4.278459e-03,0,
    7.402527e-03, 4.351614e-03, 9.999631e-01,0,
    0,0,0,1;

  ros::NodeHandle nh;

  cv::namedWindow(OPENCV_WINDOW);
  cv::startWindowThread();

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_camera = it.subscribe("/kitti/camera_color_left/image_raw", 2, camera_cb);

    // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub_pc = nh.subscribe ("/kitti/velo/pointcloud", 2, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub_pc = nh.advertise<sensor_msgs::PointCloud2> ("output_pc", 10);

  pub_camera = it.advertise("camera/image", 10);

  ros::spin();
  cv::destroyWindow(OPENCV_WINDOW);
}
