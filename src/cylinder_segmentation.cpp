#include <iostream>
#include <list>
#include <cmath>
#include <algorithm>
#include <string>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/visualization_msgs/msg/marker.hpp"

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr planes_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cylinder_pub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;

std::shared_ptr<rclcpp::Node> node;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

typedef pcl::PointXYZRGB PointT;

int marker_id = 0;
float error_margin = 0.02;  // 2 cm margin for error
float target_radius = 0.11;
bool verbose = false;

std::list<geometry_msgs::msg::PointStamped> detected_cylinders;

// set up PCL RANSAC objects

// Function to calculate distance
double distance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow((x2 - x1), 2) + std::pow((y2 - y1), 2));
}

void cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // save timestamp from message
    rclcpp::Time now = (*msg).header.stamp;

    // set up PCL objects
    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::PCDWriter writer;
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    Eigen::Vector4f centroid;

    // set up pointers
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PCLPointCloud2::Ptr pcl_pc(new pcl::PCLPointCloud2);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);
    pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());

    // convert ROS msg to PointCloud2
    pcl_conversions::toPCL(*msg, *pcl_pc);

    // convert PointCloud2 to templated PointCloud
    pcl::fromPCLPointCloud2(*pcl_pc, *cloud);

    if (verbose) {
        // std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;
    }

    // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0, 10);
    pass.filter(*cloud_filtered);
    if (verbose) {
        // std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;
    }

    // Estimate point normals
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_filtered);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    // Create the segmentation object for the planar model and set all the
    // parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.03);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(cloud_normals);

    seg.segment(*inliers_plane, *coefficients_plane);
    if (verbose) {
        // std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
    }

    // Extract the planar inliers from the input cloud
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers_plane);
    extract.setNegative(false);
    extract.filter(*cloud_plane);

    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud_filtered2);
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers_plane);
    extract_normals.filter(*cloud_normals2);

    // Create the segmentation object for cylinder segmentation and set all the
    // parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.03);
    seg.setRadiusLimits(0.06, 0.17);
    seg.setInputCloud(cloud_filtered2);
    seg.setInputNormals(cloud_normals2);

    // Obtain the cylinder inliers and coefficients
    seg.segment(*inliers_cylinder, *coefficients_cylinder);

    int num_points = (int)cloud_filtered->points.size();
    int num_inliers = (*inliers_cylinder).indices.size();
    float percent_inliers = (float)num_inliers / (float)num_points;

    if(percent_inliers < 0.05) {
        return;
    }

    // return if no cylinder was detected
    int coef_size = (*coefficients_cylinder).values.size();
    if (coef_size == 0) {
        return;
    }

    float axis_direction_x = (*coefficients_cylinder).values[3];
    float axis_direction_y = (*coefficients_cylinder).values[4];
    float axis_direction_z = (*coefficients_cylinder).values[5];
    float detected_radius = (*coefficients_cylinder).values[6];

    if (std::abs(detected_radius - target_radius) > error_margin) {
        return;
    }

    if(std::abs(axis_direction_x) < 0.99 && std::abs(axis_direction_y) > 0.01 && std::abs(axis_direction_z) > 0.01) {
        return;
    }

    if (verbose) {
        std::cerr << "Cloud Points: " << num_points << std::endl;
        std::cerr << "Cylinder inliers: " << num_inliers << std::endl;
        std::cerr << "Percent inliers: " << percent_inliers << std::endl;
    }

    if (verbose) {
        std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
    }


    // extract cylinder
    extract.setInputCloud(cloud_filtered2);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
    extract.filter(*cloud_cylinder);

    // calculate marker
    pcl::compute3DCentroid(*cloud_cylinder, centroid);
    if (verbose) {
        std::cerr << "centroid of the cylindrical component: " << centroid[0] << " " << centroid[1] << " " << centroid[2] << " " << centroid[3] << std::endl;
    }

    geometry_msgs::msg::PointStamped point_camera;
    geometry_msgs::msg::PointStamped point_map;
    visualization_msgs::msg::Marker marker;
    geometry_msgs::msg::TransformStamped tss;

    // set up marker messages
    std::string toFrameRel = "map";
    std::string fromFrameRel = (*msg).header.frame_id;
    point_camera.header.frame_id = fromFrameRel;

    point_camera.header.stamp = now;
    point_camera.point.x = centroid[0];
    point_camera.point.y = centroid[1];
    point_camera.point.z = centroid[2];

    try {
        tss = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, now);
        tf2::doTransform(point_camera, point_map, tss);
    } catch (tf2::TransformException& ex) {
        std::cout << ex.what() << std::endl;
    }

    if (verbose) {
        std::cerr << "point_camera: " << point_camera.point.x << " " << point_camera.point.y << " " << point_camera.point.z << std::endl;
        std::cerr << "point_map: " << point_map.point.x << " " << point_map.point.y << " " << point_map.point.z << std::endl;
    }

    for(auto point: detected_cylinders) {
        double dist = distance(point_map.point.x, point_map.point.y, point.point.x, point.point.y);

        if(dist < 1) {
            return;
        }
    }

    int r_sum = 0;
    int g_sum = 0;
    int b_sum = 0;

    for(int nIndex = 0; nIndex < (int)cloud_cylinder->points.size(); nIndex++) {

        int r = cloud_cylinder->points[nIndex].r;
        int g = cloud_cylinder->points[nIndex].g;
        int b = cloud_cylinder->points[nIndex].b;

        // std::cerr << r << " " << g << " " << b << std::endl;
        
	    r_sum = r_sum + r;
	    g_sum = g_sum + g;
	    b_sum = b_sum + b;
    }

    int r_avg = (int)((float)r_sum / (float)cloud_cylinder->points.size());
    int g_avg = (int)((float)g_sum / (float)cloud_cylinder->points.size());
    int b_avg = (int)((float)b_sum / (float)cloud_cylinder->points.size());

    int c_high = std::max(r_avg, std::max(g_avg, b_avg));
    int c_low = std::min(r_avg, std::min(g_avg, b_avg));
    int c_rng = c_high - c_low;

    float r1 = (float)(c_high - r_avg) / (float)c_rng;
    float g1 = (float)(c_high - g_avg) / (float)c_rng;
    float b1 = (float)(c_high - b_avg) / (float)c_rng;

    float h1;
    if(r_avg == c_high) {
        h1 = b1 - g1;
    } else if(g_avg == c_high) {
        h1 = r1 - b1 + 2.0;
    } else if(b_avg == c_high) {
        h1 = g1 - r1 + 4.0;
    }

    float h;
    if(c_rng == 0) {
        h = 0;
    } else if(h1 < 0) {
        h = (h1 + 6.0) / 6.0;
    } else {
        h = h1 / 6.0;
    }

    h = h * 360.0;

    float s = 0.0;
    if(c_high > 0) {
        s = (float)c_rng / (float)c_high;
    }

    float v = (float)c_high / 255.0;

    std::string color = "";
    if(s < 0.15) {
        return;
    } else if(v < 0.1) {
        color = "black";
    } else if(h < 15 || h > 350) {
        color = "red";
    } else if(h > 20 && h < 65) {
        if(s < 0.5) {
            return;
        }
        color = "yellow";
    } else if(h > 65 && h < 150) {
        color = "green";
    } else if(h > 180 && h < 265) {
        color = "blue";
    } else {
        return;
    }

    std::cerr << "XYZ: " << point_map.point.x << " " << point_map.point.y << " " << point_map.point.z << std::endl;
    std::cerr << "RGB: " << r_avg << " " << g_avg << " " << b_avg << std::endl;
    std::cerr << "HSV: " << h << " " << s << " " << v << std::endl;
    std::cerr << "Color: " << color << std::endl;

    detected_cylinders.push_back(point_map);

    // publish marker
    marker.header.frame_id = "map";
    marker.header.stamp = now;

    marker.ns = "cylinder";
    // marker.id = 0; // only latest marker
    marker.id = marker_id++;  // generate new markers

    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = point_map.point.x;
    marker.pose.position.y = point_map.point.y;
    marker.pose.position.z = point_map.point.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.r = (float)r_avg / 255.0;
    marker.color.g = (float)g_avg / 255.0;
    marker.color.b = (float)b_avg / 255.0;
    marker.color.a = 1.0f;

    marker_pub->publish(marker);

    // marker.lifetime = rclcpp::Duration(1,0);
    // marker.lifetime = rclcpp::Duration(10, 0);

    //////////////////////////// publish result point clouds /////////////////////////////////

    // convert to pointcloud2, then to ROS2 message
    sensor_msgs::msg::PointCloud2 plane_out_msg;
    pcl::PCLPointCloud2::Ptr outcloud_plane(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*cloud_plane, *outcloud_plane);
    pcl_conversions::fromPCL(*outcloud_plane, plane_out_msg);
    planes_pub->publish(plane_out_msg);

    // publish cylinder
    sensor_msgs::msg::PointCloud2 cylinder_out_msg;
    pcl::PCLPointCloud2::Ptr outcloud_cylinder(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*cloud_cylinder, *outcloud_cylinder);
    pcl_conversions::fromPCL(*outcloud_cylinder, cylinder_out_msg);
    cylinder_pub->publish(cylinder_out_msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::cout << "cylinder_segmentation" << std::endl;

    node = rclcpp::Node::make_shared("cylinder_segmentation");

    // create subscriber
    node->declare_parameter<std::string>("topic_pointcloud_in", "/oakd/rgb/preview/depth/points");
    std::string param_topic_pointcloud_in = node->get_parameter("topic_pointcloud_in").as_string();
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pclSubscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(param_topic_pointcloud_in, 10, &cloud_cb);

    // setup tf listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // create publishers
    planes_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("planes", 1);
    cylinder_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("cylinder", 1);
    marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("detected_cylinder", 1);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
