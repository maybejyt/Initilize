#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>



class Initilize
{
  public:
      Initilize();
      Estimator estimator;
      std::condition_variable con;
      double current_time = -1;
      queue<sensor_msgs::ImuConstPtr> imu_buf;
      queue<sensor_msgs::PointCloudConstPtr> feature_buf;
      std::mutex m_posegraph_buf;
      queue<int> optimize_posegraph_buf;
      //queue<KeyFrame*> keyframe_buf;
      queue<RetriveData> retrive_data_buf;
      std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
      std::thread measurement_process;

      std::mutex m_buf;
      std::mutex m_state;
      std::mutex i_buf;
    
      int sum_of_wait = 0;
      int sum_of_wait_= 0;
      double latest_time;
      Eigen::Vector3d tmp_P;
      Eigen::Quaterniond tmp_Q;
      Eigen::Vector3d tmp_V;
      Eigen::Vector3d tmp_Ba;
      Eigen::Vector3d tmp_Bg;
      Eigen::Vector3d acc_0;
      Eigen::Vector3d gyr_0;
      Eigen::Vector3d relocalize_t{Eigen::Vector3d(0, 0, 0)};
      Eigen::Matrix3d relocalize_r{Eigen::Matrix3d::Identity()};

      queue<pair<cv::Mat, double>> image_buf;
     

      int global_frame_cnt = 0;
     
      
      vector<int> erase_index;
      std_msgs::Header cur_header;
      
      
      void send_imu(const sensor_msgs::ImuConstPtr &imu_msg);
      void process();
      void predict(const sensor_msgs::ImuConstPtr &imu_msg);
      std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> getMeasurements();
      void Initilize_process();
      void imu_message(sensor_msgs::ImuConstPtr &imu_msg);
      void feature_message(sensor_msgs::PointCloudConstPtr &feature_msg);
      void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg);
      void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg);
      void update();
};