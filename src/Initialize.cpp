#include "Initialize.h"

Initilize::Initilize()
{
    //readParameters();
    measurement_process = std::thread(&Initilize::process,this);
     
}
void Initilize::update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = relocalize_r * estimator.Ps[WINDOW_SIZE] + relocalize_t;
    tmp_Q = relocalize_r * estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());

}

void Initilize::send_imu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    if (current_time < 0)
        current_time = t;
    double dt = t - current_time;
    current_time = t;

    double ba[]{0.0, 0.0, 0.0};
    double bg[]{0.0, 0.0, 0.0};

    double dx = imu_msg->linear_acceleration.x - ba[0];
    double dy = imu_msg->linear_acceleration.y - ba[1];
    double dz = imu_msg->linear_acceleration.z - ba[2];

    double rx = imu_msg->angular_velocity.x - bg[0];
    double ry = imu_msg->angular_velocity.y - bg[1];
    double rz = imu_msg->angular_velocity.z - bg[2];
    //ROS_DEBUG("IMU %f, dt: %f, acc: %f %f %f, gyr: %f %f %f", t, dt, dx, dy, dz, rx, ry, rz);

    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
}

void Initilize::feature_message(sensor_msgs::PointCloudConstPtr& feature_msg)
{
    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}

void Initilize::imu_message(sensor_msgs::ImuConstPtr& imu_msg)
{
    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();
    

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "world";
      
    }
}
void Initilize::feature_callback(const sensor_msgs::PointCloudConstPtr& feature_msg)
{
    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}

void Initilize::imu_callback(const sensor_msgs::ImuConstPtr& imu_msg)
{
    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();
    

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "world";
    }
}

std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> Initilize::getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
    while (true)
    {   
        
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;

        if (!(imu_buf.back()->header.stamp > feature_buf.front()->header.stamp))
        { 
           // ROS_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }
              
        if (!(imu_buf.front()->header.stamp < feature_buf.front()->header.stamp))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        feature_buf.pop();
       
        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while (imu_buf.front()->header.stamp <= img_msg->header.stamp)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

void Initilize::process()
{
   
    while (true)
    {
        measurements = getMeasurements();
        if(measurements.empty())
	     continue;
        for (auto &measurement : measurements)
        {
            for (auto &imu_msg : measurement.first)
                send_imu(imu_msg);

            auto img_msg = measurement.second;
            ROS_DEBUG("processing vision data with stamp %f \n", img_msg->header.stamp.toSec());

            TicToc t_s;
            map<int, vector<pair<int, Vector3d>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                ROS_ASSERT(z == 1);
                image[feature_id].emplace_back(camera_id, Vector3d(x, y, z));
            } 
           
            estimator.processImage(image, img_msg->header);
            /**
            *** start build keyframe database for loop closure
            **/
	    /*
	    Vector3d correct_t;
            Vector3d correct_v;
            Quaterniond correct_q;
	    Vector3d correct_bias_w;
	    Vector3d correct_bias_a;
	    correct_t = estimator.Ps[WINDOW_SIZE] ;
            correct_q = estimator.Rs[WINDOW_SIZE];
            correct_v = estimator.Vs[WINDOW_SIZE];
	    correct_bias_w = estimator.Bgs[WINDOW_SIZE];
	    correct_bias_a = estimator.Bas[WINDOW_SIZE];
	    string VINS_RESULT = "/home/jiangyuntian/catkin_ws/src/VINS-Mono-master/config/euroc/vins_result_.csv";
	    ofstream foutC(VINS_RESULT, ios::app);
            foutC.setf(ios::fixed, ios::floatfield);
            foutC.precision(0);
            foutC << img_msg->header.stamp.toSec() * 1e9 << ",";
            foutC.precision(5);
            foutC << correct_t.x() << ","
              << correct_t.y() << ","
              << correct_t.z() << ","
              << correct_q.w() << ","
              << correct_q.x() << ","
              << correct_q.y() << ","
              << correct_q.z() << ","
              << correct_v(0) << ","
              << correct_v(1) << ","
              << correct_v(2) << "," 
	      << correct_bias_w(0) <<","
	      << correct_bias_w(1) <<","
	      << correct_bias_w(2) <<","
	      << correct_bias_a(0) <<","
	      << correct_bias_a(1) <<","
	      << correct_bias_a(2) << endl;
            foutC.close();
            */
            
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
       }
    }
        
}

void Initilize::predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};
     
    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba - tmp_Q.inverse() * estimator.g);
   
    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);
     
    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba - tmp_Q.inverse() * estimator.g);
     
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    
    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;
     
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}


