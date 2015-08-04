#include <iostream>
#include "glog/logging.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "util.h"
#include "distortionImageNode.h"

using namespace std;

class distortionRosNode
{
public:
    distortionRosNode()
        : nh_ ("~")
    {
        cloud_input.reset(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_sub = nh_.subscribe<sensor_msgs::PointCloud2> ("/head_camera/depth/points", 1, \
                                                             &distortionRosNode::cloud_cb, this);
        din.cloud_size = 640*480;
        din.setSize();
    }

    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& msg_in)
    {
        pcl::fromROSMsg(*msg_in, *cloud_input);
        std::cout << cloud_input->width << " " << cloud_input->height << std::endl;
        din.cloud_in = cloud_input;
        din.process();
    }

    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input;
    distortionImageNode din;
    Eigen::Matrix4f transform;
};


int main(int argc, char** argv)
{
    //google::InitGoogleLogging(argv[0]);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    //pcl::PointCloud<pcl::PointXYZ>::Ptr corrected_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    //distortionImageNode din;
    ros::init (argc, argv, "distortion_image");
    distortionRosNode drn;
    while (ros::ok())
    {
        ros::spinOnce();
    }
    drn.din.generateResults();
    
    //std::stringstream yaml_name;
    //yaml_name << "/tmp/distortion_.yaml";
    std::ofstream file;
    file.open("/etc/ros/indigo/distortion_check.yaml");
    //std::string cal = b.printCalibrationData();
   // file.write(reinterpret_cast<char*> (&drn.din.params[0]), drn.din.params.size()*sizeof(double));
for (size_t i=0; i<drn.din.params.size(); i++)
    {
        file << drn.din.params[i] << std::endl;
    }    

file.close();
    //std::cout << cal;

    
    
//    din.cloud_size = 30000;
//    cloud->width  = 30000;
//    cloud->height = 1;
//    cloud->is_dense = false;
//    cloud->points.resize(cloud->width * cloud->height);
//    std::stringstream debug_msg;
//    debug_msg << "In debugging mode" << std::endl;

//    util::printDebug(debug_msg);

//    debug_msg << "Original Cloud" << std::endl;
//    util::printDebug(debug_msg);
//    for (size_t i = 0; i < cloud->points.size (); ++i)
//    {
//      cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
//      cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
//      cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
//    }

//    Eigen::Matrix4f transform_ = Eigen::Matrix4f::Identity();
//    transform_(2,3) = 1.5;

//    din.cloud_in  = cloud;
//    din.transform = transform_;
//    din.inv_transform = transform_.inverse();
//    din.process();
//    din.generateResults();

    std::cout << "Done!" <<std::endl;
    return 0;
}

