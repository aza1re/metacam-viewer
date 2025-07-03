#include "main.h"

int saved_count_0 = 0;
int saved_count_1 = 0;

void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    static bool init=0;

    double x, y, z;
    double roll, pitch, yaw;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    z = msg->pose.pose.position.z;
    tf::Quaternion quat;                            
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    Eigen::Affine3f Affine = pcl::getTransformation(x, y, z, roll, pitch, yaw);
    Eigen::Matrix4f matrix = Affine.matrix();

 
    poses_list.push_back(matrix);
    poses_time.push_back(msg->header.stamp.toSec());
    // ROS_INFO("Odom callback: %f, %f, %f, %f, %f, %f", x, y, z, roll, pitch, yaw);
    lidar_odoms.push_back(msg);
}


void imageCallback_0(const sensor_msgs::CompressedImage::ConstPtr& msg)
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

    // 调整分辨率
    cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(resolution[0][0], resolution[0][1]));

    // // 去畸变
    // cv::Mat undistortImg;
    // cv::remap(cv_ptr->image, undistortImg, undistor_map[0][0], undistor_map[0][1], cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    // // 只保存去畸变后的图片
    // std::string img_dir = "/home/frankye/Projects/fisheye_ws/output/images/camera0";
    // if (!dir_created) {
    //     system(("mkdir -p " + img_dir).c_str());
    //     dir_created = true;
    // }
    // char buf[32];
    // snprintf(buf, sizeof(buf), "frame_%04d", saved_count_0);
    // std::string img_name = img_dir + "/" + buf + ".png";
    // cv::imwrite(img_name, undistortImg);

    // // 保存外参
    // std::string extr_dir = "/home/frankye/Projects/fisheye_ws/output/extrinsics";
    // snprintf(buf, sizeof(buf), "frame_%04d", saved_count_0);
    // std::string yaml_name = extr_dir + "/" + buf + ".yaml";
    // Eigen::Matrix4f ext = TL2C[0];
    // Eigen::Vector3f t = ext.block<3,1>(0,3);
    // Eigen::Matrix3f R = ext.block<3,3>(0,0);
    // Eigen::Quaternionf q(R);
    // std::ofstream fout(yaml_name);
    // fout << "camera0:\n";
    // fout << "  position: [" << t(0) << ", " << t(1) << ", " << t(2) << "]\n";
    // fout << "  rotation: [" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << "]\n";
    // fout.close();

    // saved_count_0++;

    // 后续流程仍然用resize后的原图
    camera_list_0.push_back(cv_ptr->image);
    camera_time_0.push_back(msg->header.stamp.toSec());
    sensor_msgs::ImagePtr image_msg = cv_ptr->toImageMsg();
    image_msg->header = msg->header;    
    image0_msgs.push_back(image_msg);
}
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

    // 调整分辨率
    cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(resolution[1][0], resolution[1][1]));

    // // 去畸变
    // cv::Mat undistortImg;
    // cv::remap(cv_ptr->image, undistortImg, undistor_map[1][0], undistor_map[1][1], cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    // // 只保存去畸变后的图片
    // std::string img_dir = "/home/frankye/Projects/fisheye_ws/output/images/camera1";
    // if (!dir_created) {
    //     system(("mkdir -p " + img_dir).c_str());
    //     dir_created = true;
    // }
    // char buf[32];
    // snprintf(buf, sizeof(buf), "frame_%04d", saved_count_1);
    // std::string img_name = img_dir + "/" + buf + ".png";
    // cv::imwrite(img_name, undistortImg);

    // // 保存外参
    // std::string extr_dir = "/home/frankye/Projects/fisheye_ws/output/extrinsics";
    // snprintf(buf, sizeof(buf), "frame_%04d", saved_count_1);
    // std::string yaml_name = extr_dir + "/" + buf + ".yaml";
    // Eigen::Matrix4f ext = TL2C[1];
    // Eigen::Vector3f t = ext.block<3,1>(0,3);
    // Eigen::Matrix3f R = ext.block<3,3>(0,0);
    // Eigen::Quaternionf q(R);
    // std::ofstream fout(yaml_name);
    // fout << "camera1:\n";
    // fout << "  position: [" << t(0) << ", " << t(1) << ", " << t(2) << "]\n";
    // fout << "  rotation: [" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << "]\n";
    // fout.close();

    // saved_count_1++;

    // 后续流程仍然用resize后的原图
    camera_list_1.push_back(cv_ptr->image);
    camera_time_1.push_back(msg->header.stamp.toSec());
    sensor_msgs::ImagePtr image_msg = cv_ptr->toImageMsg();
    image_msg->header = msg->header;     
    image1_msgs.push_back(image_msg);    
}
void undistortImageCallback_0(const sensor_msgs::Image::ConstPtr& msg)
{
    static bool dir_created = false;
    extern int saved_count_0; // 全局变量
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // 保存图片
    std::string img_dir = "/home/user/metacam-edu-reader/output/images/camera0";
    if (!dir_created) {
        system(("mkdir -p " + img_dir).c_str());
        dir_created = true;
    }
    char buf[32];
    snprintf(buf, sizeof(buf), "frame_%04d", saved_count_0);
    std::string img_name = img_dir + "/" + buf + ".png";
    cv::imwrite(img_name, cv_ptr->image);

    // ==== 保存相机0在世界系下的外参 ====
    std::string extr_dir = "/home/user/metacam-edu-reader/output/extrinsics0";
    std::string yaml_name = extr_dir + "/" + buf + ".yaml";
    system(("mkdir -p " + extr_dir).c_str());

    // 取当前帧的雷达位姿
    if (!poses_list.empty()) {
        Eigen::Matrix4f T_w_lidar = poses_list.front();
        Eigen::Matrix4f T_w_cam0 = T_w_lidar * TL2C[0].inverse();
        Eigen::Vector3f t = T_w_cam0.block<3,1>(0,3);
        Eigen::Matrix3f R = T_w_cam0.block<3,3>(0,0);
        Eigen::Quaternionf q(R);
        std::ofstream fout(yaml_name);
        fout << "camera0:\n";
        fout << "  position: [" << t(0) << ", " << t(1) << ", " << t(2) << "]\n";
        fout << "  rotation: [" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << "]\n";
        fout.close();
    } else {
        ROS_WARN("poses_list is empty, cannot save camera0 extrinsics for this frame.");
    }

    saved_count_0++;
}

void undistortImageCallback_1(const sensor_msgs::Image::ConstPtr& msg)
{
    static bool dir_created = false;
    extern int saved_count_1; // 全局变量
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // 保存图片
    std::string img_dir = "/home/user/metacam-edu-reader/output/images/camera1";
    if (!dir_created) {
        system(("mkdir -p " + img_dir).c_str());
        dir_created = true;
    }
    char buf[32];
    snprintf(buf, sizeof(buf), "frame_%04d", saved_count_1);
    std::string img_name = img_dir + "/" + buf + ".png";
    cv::imwrite(img_name, cv_ptr->image);

    // ==== 保存相机1在世界系下的外参 ====
    std::string extr_dir = "/home/user/metacam-edu-reader/output/extrinsics1";
    std::string yaml_name = extr_dir + "/" + buf + ".yaml";
    system(("mkdir -p " + extr_dir).c_str());

    // 取当前帧的雷达位姿
    if (!poses_list.empty()) {
        Eigen::Matrix4f T_w_lidar = poses_list.front();
        Eigen::Matrix4f T_w_cam1 = T_w_lidar * TL2C[1].inverse();
        Eigen::Vector3f t = T_w_cam1.block<3,1>(0,3);
        Eigen::Matrix3f R = T_w_cam1.block<3,3>(0,0);
        Eigen::Quaternionf q(R);
        std::ofstream fout(yaml_name);
        fout << "camera1:\n";
        fout << "  position: [" << t(0) << ", " << t(1) << ", " << t(2) << "]\n";
        fout << "  rotation: [" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << "]\n";
        fout.close();
    } else {
        ROS_WARN("poses_list is empty, cannot save camera1 extrinsics for this frame.");
    }

    saved_count_1++;
}
void standard_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<PointType>::Ptr pc_ros(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*msg, *pc_ros);
    lidar_list.push_back(pc_ros);
    lidar_time.push_back(msg->header.stamp.toSec());
    frame_id = msg->header.frame_id;
    
    lidar_msgs.push_back(msg);
}

int check_key_input() {
    fd_set readfds;
    struct timeval timeout;
    int ret;

    // 清空文件描述符集合
    FD_ZERO(&readfds);
    // 将标准输入（stdin）加入集合
    FD_SET(STDIN_FILENO, &readfds);

    // 设置超时值为0，表示不阻塞
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    // 使用 select 检查输入是否准备好
    ret = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout);

    if (ret == -1) {
        perror("select()");
        exit(EXIT_FAILURE);
    }

    // 如果 stdin 可读，返回 1
    if (FD_ISSET(STDIN_FILENO, &readfds)) {
        return getchar();  // 返回输入字符
    }

    return -1;  // 没有输入，返回 -1
}

pcl::PointCloud<pcl::PointXYZRGB> all_pc;

// 新增全局变量
int save_count = 1000; // 总共保存10组
int saved_rgb_count = 0;
int rgb_save_interval = 300; // 每组10帧
int rgb_frame_counter = 0;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);


int main(int argc, char* argv[]) {

    ros::init(argc, argv, "lidar_add_rgb");
    ros::NodeHandle nh;
    nh.param<std::string>("topic_lidar_in", topic_lidar_in, "");
    nh.param<std::string>("topic_odom_in", topic_odom_in, "");        
    nh.param<std::string>("topic_lidar_out", topic_lidar_out, "");        
    nh.param<std::string>("save_pcd", save_pcd, "");        
    nh.param<int>("down_size", down_size, 1);        
    nh.param<bool>("save_rosbag", save_rosbag, true);        
    nh.param<bool>("save_pcl", save_pcl, true);        
    
    
    
    nh.param<bool>("save_assessment", save_assessment, false);        
    nh.param<std::string>("save_assessment_dir", save_assessment_dir, "xx");        
    nh.param<std::string>("save_rosbag_dir", save_rosbag_dir, "xx");            
    nh.param<int>("method", method, 0);            
    


    std::vector<double> vector_buf;   
    Eigen::Matrix4d Matrix4d_buf;

    nh.param<std::vector<double>>("lidar0/T_imu_lidar", vector_buf, std::vector<double>());
    Matrix4d_buf << vector_buf[0],vector_buf[1],vector_buf[2],vector_buf[3],  vector_buf[4],vector_buf[5],vector_buf[6],vector_buf[7],  vector_buf[8],vector_buf[9],vector_buf[10],vector_buf[11],  vector_buf[12],vector_buf[13],vector_buf[14],vector_buf[15];
    I_L.insert({0,Matrix4d_buf});

    all_pc.clear();
    merged_cloud->clear();
    saved_rgb_count = 0;
    rgb_frame_counter = 0;

    int num_camera;
    nh.param<int>("max_cameras", num_camera, 2);
    for(int i=0; i<num_camera; i++){
        topic_camera_in.push_back(" ");
        distortion_coeffs.push_back(std::vector<double>());
        intrinsics.push_back(std::vector<double>());        
        resolution.push_back(std::vector<int>());
 
        std::string cam_id = std::to_string(i);
        nh.param<std::string>("cam"+cam_id+"/rostopic",topic_camera_in[i], " ");
        nh.param<std::vector<double>>("cam"+cam_id+"/distortion_coeffs",distortion_coeffs[i], std::vector<double>());
        nh.param<std::vector<double>>("cam"+cam_id+"/intrinsics",intrinsics[i], std::vector<double>());
        nh.param<std::vector<int>>("cam"+cam_id+"/resolution",resolution[i], std::vector<int>());
        nh.param<std::vector<double>>("cam"+cam_id+"/T_lidar_cam", vector_buf, std::vector<double>());

        double k1,k2;
        k1 = ((double)resolution[i][0])/((double)3040.0);
        k2 = ((double)resolution[i][1])/((double)4032.0);      

        Eigen::Matrix4d L2C;
        L2C << vector_buf[0],vector_buf[1],vector_buf[2],vector_buf[3],  vector_buf[4],vector_buf[5],vector_buf[6],vector_buf[7],  vector_buf[8],vector_buf[9],vector_buf[10],vector_buf[11],  vector_buf[12],vector_buf[13],vector_buf[14],vector_buf[15];
        TL2C.push_back(L2C.cast<float>());     
        
        Eigen::Matrix4d C2L = Eigen::Matrix4d::Identity();
        C2L.block(0, 0, 3, 3) = L2C.block(0, 0, 3, 3).transpose();
        C2L.block(0, 3, 3, 1) = -L2C.block(0, 0, 3, 3).transpose() * L2C.block(0, 3, 3, 1);
        TC2L.push_back(C2L.cast<float>());

        std::cout<<"cam id:"<<i<<" TL2C:"<<std::endl<<L2C<<std::endl<<std::endl;
        std::cout<<"cam id:"<<i<<" TC2L:"<<std::endl<<C2L<<std::endl<<std::endl;

        std::cout<<"cam id:"<<i<<" TI2C:"<<std::endl<<L2C *   I_L.at(0)<<std::endl<<std::endl;        
        Eigen::Matrix4d I2C = L2C *   I_L.at(0);
        Eigen::Matrix4d C2I = Eigen::Matrix4d::Identity();
        C2I.block(0, 0, 3, 3) = I2C.block(0, 0, 3, 3).transpose();
        C2I.block(0, 3, 3, 1) = -I2C.block(0, 0, 3, 3).transpose() * I2C.block(0, 3, 3, 1);
        std::cout<<"cam id:"<<i<<" TC2I:"<<std::endl<<C2I<<std::endl<<std::endl;     


        camera_matrix.push_back((cv::Mat_<double>(3, 3) << intrinsics.at(i)[0]*k1, 0.0, intrinsics.at(i)[2]*k1, 0.0, intrinsics.at(i)[1]*k2, intrinsics.at(i)[3]*k2, 0.0, 0.0, 1.0));
        D.push_back((cv::Mat_<double>(1, 4) << distortion_coeffs.at(i)[0], distortion_coeffs.at(i)[1], distortion_coeffs.at(i)[2], distortion_coeffs.at(i)[3]));        

    }



    sub_odom = nh.subscribe<nav_msgs::Odometry>(topic_odom_in, 10000, &OdomCallback );
    sub_lidar = nh.subscribe<sensor_msgs::PointCloud2>(topic_lidar_in, 10000, &standard_handler );    
    sub_camera_0 = nh.subscribe<sensor_msgs::CompressedImage>(topic_camera_in[0], 10000, &imageCallback_0 );
    sub_camera_1 = nh.subscribe<sensor_msgs::CompressedImage>(topic_camera_in[1], 10000, &imageCallback_1 ); 
    ros::Subscriber sub_undistort_0 = nh.subscribe("/camera/left/jpeg_1k/undistort", 100, undistortImageCallback_0);
    ros::Subscriber sub_undistort_1 = nh.subscribe("/camera/right/jpeg_1k/undistort", 100, undistortImageCallback_1);   
    pub_lidar  = nh.advertise<sensor_msgs::PointCloud2>(topic_lidar_out, 100000);


    cv::Mat r_vec = (cv::Mat_<double>(3, 1) << 0, 0, 0);
    cv::Mat t_vec = (cv::Mat_<double>(3, 1) << 0, 0, 0);


    
  std::string path_to_bag = "camera.bag";
  path_to_bag = save_rosbag_dir + "/" + path_to_bag;
  rosbag::Bag bag;

    std::cout<<"init finsh"<<std::endl; 



    ros::Rate rate(100);
    long index=0;
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
        bool pub_state=0;  

        while(poses_time.size() && lidar_time.size()>1 && (camera_time_0.size()>1 && camera_time_1.size()>1 ) && !pub_state){

            if(camera_time_0[0]>camera_time_0[1]){                  
                camera_list_0.pop_front();
                image0_msgs.pop_front();                            
                camera_time_0.erase(camera_time_0.begin());          
            }
            else if(camera_time_1[0]>camera_time_1[1]){                 
                camera_list_1.pop_front();
                image1_msgs.pop_front();                            
                camera_time_1.erase(camera_time_1.begin());          
            }
            else if(abs(poses_time[0]-lidar_time[0]) > abs(poses_time[0]-lidar_time[1])){

                lidar_list.pop_front();
                lidar_msgs.pop_front();
                lidar_time.erase(lidar_time.begin());
            }
            else if(abs(poses_time[0]-camera_time_0[0]) > abs(poses_time[0]-camera_time_0[1])){  
                
                camera_list_0.pop_front();
                image0_msgs.pop_front();
                camera_time_0.erase(camera_time_0.begin());                
            }
            else if(abs(poses_time[0]-camera_time_1[0]) > abs(poses_time[0]-camera_time_1[1])){
                  
                camera_list_1.pop_front();
                image1_msgs.pop_front();                
                camera_time_1.erase(camera_time_1.begin());                
            }            
            else if(abs(poses_time[0]-camera_time_0[0]) >0.05|| abs(poses_time[0]-camera_time_1[0]) >0.05 || abs(poses_time[0]-lidar_time[0]) >0.05){
                poses_list.pop_front();
                lidar_odoms.pop_front();
                poses_time.erase(poses_time.begin());
                break;
            }
            else{

                if(save_rosbag){
                    auto record_time = ros::Time(lidar_odoms.front()->header.stamp.toSec());
                    if (!bag.isOpen())
                    {
                    bag.open(path_to_bag, rosbag::BagMode::Write);
                    assert(bag.isOpen());
                    }

                    auto header = lidar_odoms.front()->header;

                    auto image0_msgs_ =* image0_msgs.front();
                    image0_msgs_.header = header;
                    auto image1_msgs_ =* image1_msgs.front();
                    image1_msgs_.header = header;
                    auto lidar_msgs_ =* lidar_msgs.front();
                    lidar_msgs_.header = header;                    

                    
                    double x, y, z;
                    double roll, pitch, yaw;

                    x = lidar_odoms.front()->pose.pose.position.x;
                    y = lidar_odoms.front()->pose.pose.position.y;
                    z = lidar_odoms.front()->pose.pose.position.z;
                    tf::Quaternion quat;                            
                    tf::quaternionMsgToTF( lidar_odoms.front()->pose.pose.orientation, quat);
                    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
                    Eigen::Affine3f Affine = pcl::getTransformation(x, y, z, roll, pitch, yaw);
                    Eigen::Matrix4f matrix_new = Affine.matrix();

                    nav_msgs::Odometry image0_odom;
                    nav_msgs::Odometry image1_odom;
                    image0_odom.header = header;
                    image1_odom.header = header;         
                               
                    Eigen::Matrix4f image_0 = matrix_new * TL2C[0].inverse() ;
                    Eigen::Matrix4f image_1 = matrix_new * TL2C[1].inverse() ;         

                    Eigen::Vector3f position_0 = image_0.block<3, 1>(0, 3); // 位置部分
                    Eigen::Quaternionf orientation_0(image_0.block<3, 3>(0, 0)); // 方向部分
                    
                    Eigen::Vector3f position_1 = image_1.block<3, 1>(0, 3); // 位置部分
                    Eigen::Quaternionf orientation_1(image_1.block<3, 3>(0, 0)); // 方向部分
                
                    image0_odom.pose.pose.position.x = position_0(0);
                    image0_odom.pose.pose.position.y = position_0(1);
                    image0_odom.pose.pose.position.z = position_0(2);
                    image0_odom.pose.pose.orientation.x = orientation_0.x();
                    image0_odom.pose.pose.orientation.y = orientation_0.y();
                    image0_odom.pose.pose.orientation.z = orientation_0.z();
                    image0_odom.pose.pose.orientation.w = orientation_0.w();


                    image1_odom.pose.pose.position.x = position_1(0);
                    image1_odom.pose.pose.position.y = position_1(1);
                    image1_odom.pose.pose.position.z = position_1(2);
                    image1_odom.pose.pose.orientation.x = orientation_1.x();
                    image1_odom.pose.pose.orientation.y = orientation_1.y();
                    image1_odom.pose.pose.orientation.z = orientation_1.z();
                    image1_odom.pose.pose.orientation.w = orientation_1.w();

//                    bag.write("image0_raw", record_time, image0_msgs_);
                    bag.write("image0_odom", record_time, image0_odom);
//                    bag.write("image1_raw", record_time, image1_msgs_);
                    bag.write("image1_odom", record_time, image1_odom);
                    bag.write("lidar_odom", record_time, lidar_odoms.front());        
                    bag.write("lidar_points", record_time, lidar_msgs_);
                    image0_msgs.pop_front();
                    image1_msgs.pop_front(); 
                    lidar_odoms.pop_front();
                    lidar_msgs.pop_front();
                }
                else
                {
                    image0_msgs.pop_front();
                    image1_msgs.pop_front();
                    lidar_odoms.pop_front();
                    lidar_msgs.pop_front();
                }

                if(save_pcl)
                {
                    save_pcds.push_back(lidar_list.front());
                    save_poses.push_back(poses_list.front());          
                    save_time.push_back(lidar_time[0]); 
                    index++;
                }

                if(index%down_size!=0){
                    poses_list.pop_front();
                    lidar_list.pop_front();
                    camera_list_0.pop_front();            
                    camera_list_1.pop_front();                   
                    poses_time.erase(poses_time.begin());
                    lidar_time.erase(lidar_time.begin());
                    camera_time_0.erase(camera_time_0.begin());
                    camera_time_1.erase(camera_time_1.begin());
                    continue;
                }
                else 
                    pub_state=1;

            }
        }

        if(pub_state){
            pub_state=0;

            // 取出当前帧点云
            pcl::PointCloud<PointType>::Ptr pc(new pcl::PointCloud<PointType>());
            pc = lidar_list.front();

            // 用于存放染色后的点云
            pcl::PointCloud<pcl::PointXYZRGB> cpc;
            cpc.reserve(pc->size());

            // 构建相机图像和时间戳的映射，便于后续点云投影染色
            std::unordered_map<size_t, cv::Mat> images;
            std::unordered_map<size_t, double> stamps;            
            images.insert(std::pair<size_t,cv::Mat>(0,camera_list_0.front()));
            stamps.insert(std::pair<size_t,double>(0,camera_time_0.front()));
            images.insert(std::pair<size_t,cv::Mat>(1,camera_list_1.front()));
            stamps.insert(std::pair<size_t,double>(1,camera_time_1.front()));

            // 遍历当前帧点云的每个点，进行投影和染色
            for (size_t i = 0; i < pc->size(); i++)
            {
                Eigen::Vector4f ping;
                ping.block<3, 1>(0, 0) = pc->at(i).getVector3fMap();  
                ping.block<1, 1>(3, 0) << 1;
                double angle = M_PI;
                int cam = 0;
                bool pub_buf=0;

                // 遍历所有相机，寻找最合适的相机进行投影
                for (auto id_image:images)
                {
                    int cam_id = id_image.first;
                    double stamp = stamps.at(cam_id);
                    Eigen::Vector4f p = TL2C[cam_id]*ping;                                        
                    
                    if(p.z() < 0.01)continue; // 点在相机后方，跳过
                    double tmp = acos(p.z() / sqrt(p.x()*p.x()+p.y()*p.y())+p.z()*p.z());       
                    if(angle/2 < tmp)continue; // 视角不合适，跳过
                    cam = cam_id;
                    angle = tmp;
                    pub_buf =1;
                }
                if(!pub_buf) continue; // 没有合适相机，跳过

                // 将点转换到选定相机坐标系
                ping = TL2C[cam]*ping;                                  
                std::vector<cv::Point3d> pts_3d;
                pts_3d.emplace_back(cv::Point3d(ping.x(), ping.y(), ping.z()));
                std::vector<cv::Point2d> pts_2d;

                // 使用鱼眼模型将3D点投影到2D图像平面
                cv::fisheye::projectPoints(pts_3d, pts_2d, r_vec, t_vec, camera_matrix[cam], D[cam]);
                cv::Point2f pp = pts_2d[0];

                // 判断投影点是否在图像范围内
                if (pp.x > 0 && pp.x < resolution[cam][0] && pp.y > 0 && pp.y < resolution[cam][1]) {
                    // 取出像素颜色，赋值给点云
                    auto cc = images.at(cam).at<cv::Vec3b>(pp);
                    pcl::PointXYZRGB p;
                    p.x = pc->at(i).x, p.y = pc->at(i).y, p.z = pc->at(i).z, p.r = cc[2], p.b = cc[0], p.g = cc[1];
                    cpc.push_back(p);
                }
            }

            // 将点云变换到世界坐标系
            pcl::transformPointCloud(cpc, cpc, poses_list.front());   

            // 累加到完整点云
            if(save_pcl)
                all_pc += cpc;

            // 合并到10帧累计点云
            *merged_cloud += cpc;
            rgb_frame_counter++;

            // 每累计10帧，保存一次合并点云
            if(saved_rgb_count < save_count && rgb_frame_counter == rgb_save_interval) {
                std::string filename = "/home/user/metacam-edu-reader/output/pointcloud/cloud_group_" + std::to_string(saved_rgb_count) + ".pcd";
                pcl::io::savePCDFileBinary(filename, *merged_cloud);
                saved_rgb_count++;
                rgb_frame_counter = 0;
                merged_cloud->clear();
                std::cout << "Saved point cloud group " << saved_rgb_count << std::endl;
            }


            // 发布染色后的点云
            pc->clear();
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(cpc, msg);
            msg.header.stamp = ros::Time(poses_time.front());
            if(method == 1)
                msg.header.frame_id = "odom";
            else
                msg.header.frame_id = frame_id;
            pub_lidar.publish(msg);

            // 移除已处理的数据，准备处理下一帧
            poses_list.pop_front();
            lidar_list.pop_front();
            camera_list_0.pop_front();            
            camera_list_1.pop_front();                        
            poses_time.erase(poses_time.begin());
            lidar_time.erase(lidar_time.begin());
            camera_time_0.erase(camera_time_0.begin());
            camera_time_1.erase(camera_time_1.begin());
        }
     
        char ch = check_key_input();
        if(ch=='a' && save_pcl){

        pcl::PCDWriter pcd_writer;
    //  pcd_writer.writeBinary(save_pcd, all_pc);
        bool binary_mode = false;
        if(-1 == pcl::io::savePCDFile(save_pcd, all_pc, binary_mode)){
            std::cout<<"save pcd file failed."<<std::endl;
            return -1;
        }else{
            std::cout<<"pcd file saved at "<< save_pcd <<std::endl<<std::endl;
        }
        if(save_assessment){
            for(int j =0;j<save_pcds.size();j++){
                std::string index_ = std::to_string(j);
                pcl::io::savePCDFileBinary(save_assessment_dir + "/" + index_ + ".pcd", *save_pcds[j]);
            }
            std::ofstream  file_pose_optimized;
            std::string pose_file (save_assessment_dir+ "/poses.txt");
            file_pose_optimized.open(pose_file, std::ios::out);                          //  使用std::ios::out 可实现覆盖
            if(!file_pose_optimized){
                std::cout << "can not open  file to save pose " << std::endl;
            }                
            for(int j =0;j<save_poses.size();j++){
                Eigen::Matrix3f R = save_poses[j].block(0,0,3,3).matrix();
                Eigen::Quaternionf quaternion(R);
                file_pose_optimized << std::fixed  
                                                            <<  save_time[j]  << " " << save_poses[j](0,3)   << " "<<   save_poses[j](1,3)  << " "  <<    save_poses[j](2,3)  <<  " "
                                                            <<  quaternion.x()  << " " << quaternion.y()   << " "<<   quaternion.z()  << " "  <<    quaternion.w()  
                                                            <<  std::endl;                        
            }
        }

        }
    }
    return 0;

}
