#include "main.h"
 
// 全局队列，用于存储相机和雷达的时间戳、点云和消息头
std::list<double> camera_time;
std::list<double> lidar_time;
std::list<pcl::PointCloud<edu_ros::Point>> lidar_pc;
std::list<std_msgs::Header> lidar_header;

// 保存图片的计数器
int save_count = 10000; // 可通过参数设置
int saved_count_0 = 0;
int saved_count_1 = 0;

// IMU回调，单位转换并发布
void IMUCallBack(const sensor_msgs::Imu::ConstPtr &msg_in) {
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
    msg->linear_acceleration.x*=g;
    msg->linear_acceleration.y*=g;
    msg->linear_acceleration.z*=g;        
    pub_imu.publish(*msg);
    return; 
}

// 相机0图像回调，解码、去畸变、发布、保存前100帧原图和去畸变图
void imageCallback_0(const sensor_msgs::CompressedImage::ConstPtr& msg)
{
    static bool dir_created = false;
    if(pub_lidar_time_align.getNumSubscribers() != 0)
        camera_time.push_back(msg->header.stamp.toSec());

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // 调整图像分辨率
    cv::Mat resizedImage;
    cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(out_resolution[0][0], out_resolution[0][1]));    
    // 发布原始图像
    sensor_msgs::ImagePtr image_msg = cv_ptr->toImageMsg();
    image_msg->header = msg->header;    
    pub_camera_0.publish(image_msg);
    // 去畸变
    cv::Mat undistortImg;
    cv::remap(cv_ptr->image, undistortImg, undistor_map[0][0], undistor_map[0][1], cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    // 发布去畸变图像
    sensor_msgs::ImagePtr  image_msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", undistortImg).toImageMsg();
    image_msg_->header = msg->header;    
    pub_camera_undistort_0.publish(image_msg_);

    // // 保存前100帧原图和去畸变图像
    // if (saved_count_0 < save_count) {
    //     // 创建目录
    //     std::string img_dir = "/home/frankye/Projects/fisheye_ws/output/images/camera0";
    //     if (!dir_created) {
    //         system(("mkdir -p " + img_dir).c_str());
    //         dir_created = true;
    //     }
    //     char buf[32];
    //     snprintf(buf, sizeof(buf), "frame_%04d", saved_count_0);
    //     std::string img_name = img_dir + "/" + buf + ".png";
    //     // 保存图片
    //     cv::imwrite(img_name, undistortImg);
    //     saved_count_0++;
    // }
}

// 相机1图像回调，解码、去畸变、发布、保存前100帧原图和去畸变图
void imageCallback_1(const sensor_msgs::CompressedImage::ConstPtr& msg)
{
    static bool dir_created = false;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // 调整图像分辨率
    cv::Mat resizedImage;
    cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(out_resolution[1][0], out_resolution[1][1]));    
    // 发布原始图像
    sensor_msgs::ImagePtr image_msg = cv_ptr->toImageMsg();
    image_msg->header = msg->header;    
    pub_camera_1.publish(image_msg);
    // 去畸变
    cv::Mat undistortImg;
    cv::remap(cv_ptr->image, undistortImg, undistor_map[1][0], undistor_map[1][1], cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    // 发布去畸变图像
    sensor_msgs::ImagePtr  image_msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", undistortImg).toImageMsg();
    image_msg_->header = msg->header;    
    pub_camera_undistort_1.publish(image_msg_);

    // // 保存前100帧原图和去畸变图像
    // if (saved_count_1 < save_count) {
    //     std::string img_dir = "/home/frankye/Projects/fisheye_ws/output/images/camera1";
    //     if (!dir_created) {
    //         system(("mkdir -p " + img_dir).c_str());
    //         dir_created = true;
    //     }
    //     char buf[32];
    //     snprintf(buf, sizeof(buf), "frame_%04d", saved_count_1);
    //     std::string img_name = img_dir + "/" + buf + ".png";
    //     cv::imwrite(img_name, undistortImg);
    //     saved_count_1++;
    // }
}

// 雷达回调，将PointCloud2转为自定义点云类型，发布自定义消息
void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // 转换为自定义PCL点云类型
    pcl::PointCloud<edu_ros::Point> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // 如果有订阅者，保存时间戳、点云和header到队列
    if(pub_lidar_time_align.getNumSubscribers() != 0){
        lidar_time.push_back(msg->header.stamp.toSec());
        lidar_pc.push_back(cloud);
        lidar_header.push_back(msg->header);
    }
    // 发布自定义格式的点云消息
    if(pub_lidar.getNumSubscribers() != 0){
        livox_ros_driver::CustomMsg custom_msg;
        custom_msg.header = msg->header;
        custom_msg.point_num = cloud.points.size();
        custom_msg.timebase = msg->header.stamp.toNSec();
        for (const auto& pt : cloud.points) {
            livox_ros_driver::CustomPoint custom_point;
            custom_point.x = pt.x;
            custom_point.y = pt.y;
            custom_point.z = pt.z;
            custom_point.reflectivity = static_cast<uint8_t>(pt.intensity);  // 强度映射为反射率
            custom_point.offset_time = static_cast<uint32_t>((pt.timestamp )); // 时间偏移
            custom_point.tag = pt.tag;  // 标签
            custom_point.line = pt.line;  // 激光编号
            custom_msg.points.push_back(custom_point);
        }
        pub_lidar.publish(custom_msg);
    }
}

// 主函数，参数读取、订阅与发布初始化、主循环
int main(int argc, char* argv[]) {

    ros::init(argc, argv, "data_preprocessing");
    ros::NodeHandle nh;    

    nh.param<double>("g", g, 9.80511);    

    int num_camera;
    nh.param<int>("max_cameras", num_camera, 2);

    // 读取每个相机的参数，初始化去畸变映射
    for(int i=0; i<num_camera; i++){
        topic_camera_in.push_back(" ");
        topic_camera_out.push_back(" ");
        topic_camera_undistort_out.push_back(" ");                

        distortion_coeffs.push_back(std::vector<double>());
        intrinsics.push_back(std::vector<double>());        
        resolution.push_back(std::vector<int>());
        out_resolution.push_back(std::vector<int>());

        std::string cam_id = std::to_string(i);
        nh.param<std::string>("cam"+cam_id+"/rostopic",topic_camera_in[i], "x");
        nh.param<std::string>("cam"+cam_id+"/rostopic_out",topic_camera_out[i], "x");
        nh.param<std::string>("cam"+cam_id+"/rostopic_out_undistort",topic_camera_undistort_out[i], "x");
        nh.param<std::vector<double>>("cam"+cam_id+"/distortion_coeffs",distortion_coeffs[i], std::vector<double>());
        nh.param<std::vector<double>>("cam"+cam_id+"/intrinsics",intrinsics[i], std::vector<double>());
        nh.param<std::vector<int>>("cam"+cam_id+"/resolution",resolution[i], std::vector<int>());
        nh.param<std::vector<int>>("cam"+cam_id+"/out_resolution",out_resolution[i], std::vector<int>());

        // 计算缩放比例，构造相机内参矩阵和畸变参数
        double k1,k2;
        k1 = ((double)out_resolution[i][0])/((double)3040.0);
        k2 = ((double)out_resolution[i][1])/((double)4032.0);        
        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << intrinsics[i][0]*k1, 0.0, intrinsics[i][2]*k1, 0.0, intrinsics[i][1]*k2, intrinsics[i][3]*k2, 0.0, 0.0, 1.0);
        cv::Mat distortion_coeff = (cv::Mat_<double>(1, 4) << distortion_coeffs[i][0], distortion_coeffs[i][1], distortion_coeffs[i][2], distortion_coeffs[i][3]); 

        // 初始化去畸变映射表
        cv::Mat map1, map2;
        cv::Mat undistortImg;
        cv::Size imageSize(out_resolution[i][0], out_resolution[i][1]);
        cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
        cv::Mat t_vec = (cv::Mat_<double>(3, 1) << 0, 0, 0);
        cv::fisheye::initUndistortRectifyMap(camera_matrix, distortion_coeff, cv::Mat(), camera_matrix, imageSize, CV_16SC2, map1, map2);
        std::vector<cv::Mat> map_buf;
        map_buf.push_back(map1);
        map_buf.push_back(map2);        
        undistor_map.push_back(map_buf);
    }
    // 读取IMU和雷达相关参数
    nh.param<std::string>("topic_imu_in", topic_imu_in, "");
    nh.param<std::string>("topic_imu_out", topic_imu_out, "");

    nh.param<std::string>("lidar/topic_lidar_in", topic_lidar_in, "");
    nh.param<std::string>("lidar/topic_lidar_out", topic_lidar_out, "");
    nh.param<std::string>("lidar/topic_lidar_time_align", topic_lidar_time_align, "");

    // 订阅和发布初始化
    sub_lidar = nh.subscribe<sensor_msgs::PointCloud2>(topic_lidar_in, 10000, &lidarCallback );
    // pub_lidar  = nh.advertise<sensor_msgs::PointCloud2>(topic_lidar_out, 100000);
    pub_lidar_time_align  = nh.advertise<sensor_msgs::PointCloud2>(topic_lidar_time_align, 100000);    
    pub_lidar  = nh.advertise<livox_ros_driver::CustomMsg>(topic_lidar_out, 100000);
    sub_camera_0 = nh.subscribe<sensor_msgs::CompressedImage>(topic_camera_in[0], 10000, &imageCallback_0 );
    sub_camera_1 = nh.subscribe<sensor_msgs::CompressedImage>(topic_camera_in[1], 10000, &imageCallback_1 );
        

    sub_imu = nh.subscribe<sensor_msgs::Imu>(topic_imu_in, 10000, &IMUCallBack );
    pub_imu  = nh.advertise<sensor_msgs::Imu>(topic_imu_out, 100000);

    pub_camera_0  = nh.advertise<sensor_msgs::Image>(topic_camera_out[0], 100000);
    pub_camera_undistort_0  = nh.advertise<sensor_msgs::Image>(topic_camera_undistort_out[0], 100000);

    pub_camera_1  = nh.advertise<sensor_msgs::Image>(topic_camera_out[1], 100000);
    pub_camera_undistort_1  = nh.advertise<sensor_msgs::Image>(topic_camera_undistort_out[1], 100000);    

    ros::Rate rate(200);

    double camera_last=0;
    // 主循环：实现雷达与相机时间对齐
    while (ros::ok()) {
        ros::spinOnce();

        // 如果有足够的相机和雷达数据，进行时间对齐处理
        if(camera_time.size()>0 && lidar_time.size()>1){
            if(lidar_time.front()>camera_time.front()){
                camera_last = camera_time.front();
                camera_time.pop_front();
                continue;
            }
            double camera_current = camera_time.front();

            // 合成与当前相机帧时间对齐的点云
            pcl::PointCloud<edu_ros::Point> out_buf;  
            auto lidar_ = lidar_pc.begin();    
            int num_ = (*lidar_).size();
            for(int i=0;i<num_;i++){
                if((*lidar_).points[i].timestamp * 1e-8 > camera_last){
                    out_buf.push_back((*lidar_).points[i]);
                }
            }
            ++lidar_;
            num_ = (*lidar_).size();
            for(int i=0;i<num_;i++){
                if((*lidar_).points[i].timestamp * 1e-8 < camera_current){
                    out_buf.push_back((*lidar_).points[i]);
                }
            }

            // 发布对齐后的点云
            sensor_msgs::PointCloud2 tempCloud1;
            pcl::toROSMsg(out_buf, tempCloud1);
            tempCloud1.header.stamp = ros::Time().fromSec(camera_current);
            tempCloud1.header.frame_id = lidar_header.front().frame_id;
            if (pub_lidar_time_align.getNumSubscribers() != 0)
                pub_lidar_time_align.publish(tempCloud1);

            // 移除已处理的数据
            camera_last = camera_current;
            camera_time.pop_front();
            lidar_time.pop_front();
            lidar_header.pop_front();
            lidar_pc.pop_front();
        }

        rate.sleep();
    }
    return 0;

}