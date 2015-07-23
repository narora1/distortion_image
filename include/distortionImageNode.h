#ifndef DISTORTIONIMAGENODE_H
#define DISTORTIONIMAGENODE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "ceres/ceres.h"
#include "glog/logging.h"
#include "distortionImage.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "pcl_ros/transforms.h"

using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

class distortionImageNode
{
public:
    distortionImageNode();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in;
    pcl::PointCloud<pcl::PointXYZ>::Ptr corrected_cloud;
    std::stringstream debug_msg;
    //Eigen::Matrix4f transform;
    //Eigen::Matrix4f inv_transform;
    std::vector<double> measured;
    std::vector<double> initials;
    std::vector<double> params;
    DistortionImageFunctor::DistortionImageCostFunction* cost_function;

    tf::TransformListener listener;
    tf::StampedTransform transform;
    tf::StampedTransform inv_transform;

    int cloud_size;

    Problem problem;
    Solver::Summary summary;

    void process();
    void generateResults();
    void setSize();
};

#endif // DISTORTIONIMAGENODE_H
