#include "distortionImageNode.h"

distortionImageNode::distortionImageNode()
{
    cloud_in.reset(new pcl::PointCloud<pcl::PointXYZ>);
    corrected_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

void distortionImageNode::setSize()
{
    params.resize(cloud_size);
}

void distortionImageNode::process()
{
    std::cout << cloud_in->width << std::endl;
    
    try{
        listener.lookupTransform("/head_camera_depth_optical_frame", "/base_link",
                                 ros::Time(0), transform);
        listener.lookupTransform("/base_link","/head_camera_depth_optical_frame",
                                 ros::Time(0), inv_transform);

   std::cout<<"hi"<<std::endl; 
   //std::cout<<"tranform"<< transform.pose<<std::endl;
        
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    

    //Transform to base
    pcl_ros::transformPointCloud(*cloud_in, *corrected_cloud, transform);

    for (size_t i = 0; i < corrected_cloud->points.size(); i++)
    {
        corrected_cloud->points[i].z = 0;
    }

    //Transform back to camera frame
    pcl_ros::transformPointCloud(*corrected_cloud, *corrected_cloud, inv_transform);

    std::cout << "Size: " << corrected_cloud->points.size() << std::endl;
    for (size_t i =0; i <corrected_cloud->points.size(); i ++)
    {
        measured.clear();
        initials.clear();
        measured.push_back(corrected_cloud->points[i].x);
        measured.push_back(corrected_cloud->points[i].y);
        measured.push_back(corrected_cloud->points[i].z);
        measured.push_back(cloud_in->points[i].x);
        measured.push_back(cloud_in->points[i].y);
        measured.push_back(cloud_in->points[i].z);
        initials.push_back(1.0);
        params[i] = 0.1;
        cost_function = DistortionImageFunctor::Create(measured, 0);
        problem.AddResidualBlock(cost_function, NULL, &(params[i]));
    }
}

void distortionImageNode::generateResults()
{
    Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
}
