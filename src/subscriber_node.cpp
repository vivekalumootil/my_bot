#include <chrono>
#include <memory>
#include <algorithm>
#include <iostream>
#include <vector>
#include <math.h>
#include <stdio.h>
#include <queue>
#include <set>


#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/hal/interface.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#define RAD2DEG 57.295779513
#define PI 3.1415

const double VX = 0.7; const double VY = 0.7;

typedef std::vector<std::pair<double, double>> vPT;
typedef std::vector<std::tuple<double, double, double>> cPT;
typedef std::pair<double, double> PT;
typedef std::tuple<double, double, double> CT;

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
  public:
    MinimalSubscriber()
    : Node("controls"), target_x(5), target_y(3)
    {
      auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
      // Regular Camera Subscriber
      camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", qos_profile, std::bind(&MinimalSubscriber::camera_callback, this, _1));
      
      // Depth Camera Subscriber
      depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/depth/image_raw", qos_profile, std::bind(&MinimalSubscriber::depth_callback, this, _1));
      
      // Odometry Subscriber 
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", qos_profile, std::bind(&MinimalSubscriber::odom_callback, this, _1));
      
      // LiDAR Subscriber
      laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos_profile, std::bind(&MinimalSubscriber::laser_callback, this, _1));
      
      pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud", qos_profile, std::bind(&MinimalSubscriber::pcl_callback, this, _1));
      
      // Controls (Differential Drive) Publisher
      ctr_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    }

  private :
  
    // HELPER FUNCTIONS
    
    double dist_2d(double x1, double y1, double x2, double y2) 
    {
      return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    }
    
    void find_circle(double x1, double y1, double x2, double y2, double x3, double y3, double& cx, double& cy, double& rad)
    {
      double J = (x1-x2)*(y1-y3) - (x1-x3)*(y1-y2);
      double U = x1*x1+y1*y1-x3*x3-y3*y3;
      double T = x1*x1+y1*y1-x2*x2-y2*y2;
      cx = -(y1-y2)*U + (y1-y3)*T;
      cx = cx/(2*J);
      cy = (x1-x2)*U - (x1-x3)*T;
      cy = cy/(2*J);
      rad = dist_2d(x2, y2, cx, cy);
    }
    
    static bool angle_comparison(const PT &a, const PT &b)
    {
      return get_angle(a.first, a.second) < get_angle(b.first, b.second);
    }
    
    static double get_angle(double dx, double dy)
    {
       if (dx > 0.1) {
         return atan(dy/dx);
       }
       else if (dx < 0.1) {
         return atan(dy/dx) + PI;
       }
       else {
       	 return PI/2;
       }
    }
    
    cPT find_cylinders(vPT points)
    {
    
      cPT centers;
      
      // sort by angle
      std::sort(points.begin(), points.end(), angle_comparison);
      
      // Create Mark array
      int* mark = new int[points.size()];
      for (int i=0; i<points.size(); i++) {
        if (abs(points[i].first ) <= 20 and abs(points[i].second) <= 20) {
      	  mark[i] = 0;
      	}
      }
      
      // Detect centers
      int t = points.size();
      for (int i=0; i<t-2; i++) {
        double dx; double dy; double r;
        find_circle(points[i].first, points[i].second, points[i+1].first, points[i+1].second, points[i+2].first, points[i+2].second, dx, dy, r);
        int ctr = 0;
        for (int j=0; j<points.size(); j++) {
          if (mark[j] == 0) {
            if (abs(dist_2d(points[j].first, points[j].second, dx, dy)-r) <= 0.03) {
              ctr += 1;
            }
          }
        }
        if (ctr >= 15) {
          for (int j=0; j<points.size(); j++) {
            if (abs(dist_2d(points[j].first, points[j].second, dx, dy)-r) <= 0.03) {
              mark[j] = 1;
            }
          }
          centers.push_back(CT(dx, dy, r));
        }
      }
      
      return centers;
    }
     
     
    void adjust_queue(cPT cntrs)
    {
      for (int i=0; i<cntrs.size(); i++) {
        bool valid = true; std::set<PT>::iterator itr;
        for (itr = viewed.begin(); itr != viewed.end(); itr++) {
          if (dist_2d(current_x+std::get<0>(cntrs[i]), current_y+std::get<1>(cntrs[i]), itr->first, itr->second) <= 0.2) {
            valid = false;
          }
        }
        if (valid) {
          path.push(PT(current_x+std::get<0>(cntrs[i])+VX, current_y+std::get<1>(cntrs[i])+VY));
          viewed.insert(PT(current_x+std::get<0>(cntrs[i])+VX, current_y+std::get<1>(cntrs[i])+VY));
          RCLCPP_INFO(this->get_logger(), "Inserting new target destination to queue");
        }
      }    
      if (dist_2d(current_x, current_y, path.front().first, path.front().second) <= 0.4) {
        path.pop();
      }
      target_x = path.front().first; target_y = path.front().second;
    }
    
    
    // CALLBACK FUNCTIONS
   
    void pcl_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      //RCLCPP_INFO(this->get_logger(), "Height is %d", msg->height);
      //cv::Mat drawing = cv::Mat::zeros(cv::Size(360, 480),CV_8UC3);
      cv::Mat drawing(360, 480, CV_8UC3, cv::Scalar(228, 229, 247));
      cv::circle(drawing, cv::Point(240, 180), 10, cv::Scalar(87, 93, 161), -1);
      vPT my_points;
      for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x"); it != it.end(); ++it) {
        double px = it[0]; double py = it[1];
        my_points.push_back(PT(px, py));
        cv::circle(drawing, cv::Point(50*px+240, 50*py+180), 3, cv::Scalar(171, 130, 232), -1);
      }
      
      my_centers = find_cylinders(my_points);
      
      for (int i=0; i<my_centers.size(); i++) {
        double px = std::get<0>(my_centers[i]); double py = std::get<1>(my_centers[i]); double R = std::get<2>(my_centers[i]);
        RCLCPP_INFO(this->get_logger(), "Center is x: %f and y: %f, with radius %f", px, py, R);
        cv::circle(drawing, cv::Point(50*px+240, 50*py+180), 50*R, cv::Scalar(214, 140, 43), -1);
      }
      
      adjust_queue(my_centers);
      cv::imshow("PCL DISPLAY", drawing);
      cv::waitKey(1);
    }
    
    // Currently not in use
    void laser_callback(sensor_msgs::msg::LaserScan::SharedPtr msg)
    {  
    /*
      int space = 0;
      // RCLCPP_INFO(this->get_logger(), "Size is %d", x);
      //double dx; double dy; double r; find_circle(1, 2, 3, 4, 2, 7, dx, dy, r);
      //RCLCPP_INFO(this->get_logger(), "Coordinates/rad are %f, %f, %f", dx, dy, r);
      std::vector<std::pair<double, double>> scan_map;
      double angle = msg->angle_min;
      cv::Mat drawing = cv::Mat::zeros(cv::Size(360, 480),CV_8UC3);
      for (int i=0; i<msg->ranges.size(); i++) {
        if (isinf(msg->ranges[i])) {
          //RCLCPP_INFO(this->get_logger(), "Found infinity!");
          continue;
        }
        double px = cos(angle) * msg->ranges[i];
        double py = sin(angle) * msg->ranges[i];
        //RCLCPP_INFO(this->get_logger(), "Point is %f, %f", px, py);
        cv::circle(drawing, cv::Point(50*px+180, 50*py+240), 5, cv::Scalar(0, 0, 255), -1);
        scan_map.push_back(std::pair<double, double>(px, py));
        angle += msg->angle_increment;
        space += 1;
      }
      //RCLCPP_INFO(this->get_logger(), "Space is %d", space);
      int* mark = new int[space];
      for (int i=0; i<space; i++) {
        mark[i] = 0;
      }
      std::vector<std::pair<double, double>> centers;
      for (int i=0; i<space; i++) {
        if (mark[i] != 0) {
          continue;
        }
        double dx; double dy; double r;
        find_circle(scan_map[i].first, scan_map[i].second, scan_map[i+1].first, scan_map[i+1].second, scan_map[i+2].first, scan_map[i+2].second, dx, dy, r);
        int ctr = 0;
        for (int j=0; j<space; j++) {
          if (mark[j] != 0) {
            continue;
          }
          if (abs(dist_2d(scan_map[j].first, scan_map[j].second, dx, dy)-r) <= 0.2) {
            ctr += 1;
          }
        }
        if (ctr >= 6) {
          centers.push_back(std::pair<double, double>(dx, dy));
          //cv::circle(drawing, cv::Point(50*dx+180, 50*dy+240), 10, cv::Scalar(100, 0, 255), -1);
          RCLCPP_INFO(this->get_logger(), "Center is %f, %f", dx, dy);
          for (int j=0; j<space; j++) {
            if (abs(dist_2d(scan_map[j].first, scan_map[j].second, dx, dy)-r) <= 0.1) {
              mark[j] = 1;
            }
          }
        }
      }
      
      //RCLCPP_INFO(this->get_logger(), "Number of cylinders found: %d", (int) centers.size());
      cv::imshow("Drawing", drawing);
      cv::waitKey(1);
   */
    }
    
    void camera_callback(sensor_msgs::msg::Image::SharedPtr msg)
    {
      /*
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(msg,"rgb8");
      
      // Convert image to grayscale
      cv::Mat img_gray;
      cv::cvtColor(cv_ptr->image, img_gray, cv::COLOR_BGR2GRAY);
      
      // Blur the image
      cv::Mat img_blur;
      cv::GaussianBlur(img_gray, img_blur, cv::Size(3,3), 0);
      
      // Perform Canny Edge Detection
      cv::Mat edges;
      cv::Canny(img_blur, edges, 10, 100, 3, false);
      //cv::imshow("Canny", edges);
      
      // Find all the bottom points of the near-vertical edges
      std::vector<cv::Vec4i> lines;
      cv::HoughLinesP(edges, lines, 1, 3.1415/180, 100, 10, 250);
      std::vector<int> net_points;
      for (size_t i=0; i<lines.size(); i++) {
      	cv::Vec4i l = lines[i];
      	double slope; double arc;
      	if (abs(l[2]-l[0]) < 4) {
      	  arc = 90;
      	}
      	else {
      	  slope = (l[3]-l[1])/(l[2]-l[0]);
    	  arc = atan(slope) * 180/3.1415;
      	} 
    	if (arc > 75 or arc < -75) {
    	    cv::line(img_gray, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 255, 255), 6, cv::LINE_AA);
    	  if (l[1] < 10) {
    	    net_points.push_back(l[0]);
    	    net_points.push_back(l[1]);
    	  }
    	  if (l[3] < 10) {
    	    net_points.push_back(l[2]);
    	    net_points.push_back(l[3]);
    	  }
    	}
    	else {
    	cv::line(img_gray, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    	}
      }
      //RCLCPP_INFO(this->get_logger(), "Number of points: %d", (int) net_points.size());
      straight_points = net_points;
      cv::waitKey(1);
      */
    }
    
    void depth_callback(sensor_msgs::msg::Image::SharedPtr msg)
    { 
    /*
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(msg,"32FC1");
      //cv::imshow("Depth", cv_ptr->image);
      */
    }
    void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg)
    {
      geometry_msgs::msg::Twist twist_msg = geometry_msgs::msg::Twist();
      double tx = msg->pose.pose.position.x;
      double ty = msg->pose.pose.position.y;
      double tz = msg->pose.pose.position.z;
      current_x = tx; current_y = ty;
      // RCLCPP_INFO(this->get_logger(), "%s at %.2f %.2f", msg->header.frame_id.c_str(), tx, ty);
      double dx = target_x-tx; double dy = target_y-ty; 
      double dw = sqrt(dx*dx+dy*dy);
      dx /= dw; dy /= dw;
      tf2::Quaternion q(
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);

        // 3x3 Rotation matrix from quaternion
        tf2::Matrix3x3 m(q);

        // Roll Pitch and Yaw from rotation matrix
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        roll *= RAD2DEG; pitch *= RAD2DEG; yaw *= RAD2DEG;
        //RCLCPP_INFO(get_logger(), "Yaw is %.2f", yaw);
        double traj;
        if (abs(dx) < 0.02) {
          traj = 90;
        }
        else {
          traj = atan(dy/dx) * 180/3.1415;
        }
        //RCLCPP_INFO(this->get_logger(), "Trajectory is %f", traj);
        
        
        double rec_x = cos(yaw/RAD2DEG); double rec_y = sin(yaw/RAD2DEG);
        bool valid = true;
	for (int i=0; i<my_centers.size(); i++) {
          double px = std::get<0>(my_centers[i]); double py = std::get<1>(my_centers[i]); double R = std::get<2>(my_centers[i]);
          if (dist_2d(px, py, rec_x, rec_y) <= (R+0.7)) {
            valid = false;
          }
        }
        if (valid) {
          if (abs(traj-yaw) > 1) {
            twist_msg.angular.z = abs(traj-yaw)/(7*(traj-yaw));
          } 
          else {
            twist_msg.angular.z = 0;
          }
        }
        else {
          twist_msg.angular.z = 0.1;
        } 

        twist_msg.linear.x = 0;
        if (valid) {
          if (dw >= 1) {
            twist_msg.linear.x = 0.2;
          }
          else if (dw >= 0.3) {
            twist_msg.linear.x = 0.10;
          }
          else {
            twist_msg.linear.x = 0;
          }
        }
	/*
      if (straight_points.size() > 1) {
        double mid = (straight_points[0]+straight_points[2])/2;
      	twist_msg.linear.x = 0.2;
      	if (mid > 5) {
      	  twist_msg.angular.z = 0.1;
        }
        else if (mid < -5) {
        twist_msg.angular.z = -0.1;
        }
        else {
          twist_msg.angular.z = 0;      
        }
      }
      else {
      	twist_msg.linear.x = 0;
      	 twist_msg.angular.z = 0;     
      }
      */
        ctr_pub_->publish(twist_msg);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ctr_pub_;
    std::vector<int> straight_points;
    std::queue<PT> path;
    std::set<PT> viewed;
    cPT my_centers;
    double target_x; double target_y;
    double current_x; double current_y;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
