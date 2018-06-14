#include <ros/ros.h>
#include <Eigen/Dense>
#include <iostream>
#include <set>
// PCL specific includes
#include <pcl/common/centroid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/transforms.h>
#include <boost/lexical_cast.hpp>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

// cylinder fitting

#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <visualization_msgs/MarkerArray.h>

// ros::Publisher pub;
std::vector<ros::Publisher> g_pub_vec;
ros::Publisher g_pub_cyl_cloud;
ros::Publisher  g_pub_cyl_markers;
std::vector<std::pair<Eigen::Vector4f, int> > g_last_centroids;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    ros::Time begin = ros::Time::now();

    ///@{    Transform cloud (Identity for now)
    sensor_msgs::PointCloud2 transformed_cloud;
    tf::TransformListener listener;

    while (true) {
        if (!pcl_ros::transformPointCloud(input->header.frame_id, *input, transformed_cloud, listener)) {
            ros::Duration(0.1).sleep();
        }
        else {
          break;
        }
    }
    //@}

    ///@{    Downsample
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_XYZ  (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f  (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2::Ptr downsampled (new sensor_msgs::PointCloud2);


    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;
  
    // std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
    //         << " data points." << std::endl;
    // Convert to PCL data type
    pcl_conversions::toPCL(transformed_cloud, *cloud);
  
    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;

    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (cloud_filtered);
    // Change from type sensor_msgs::PointCloud2 to pcl::PointXYZ
    pcl::fromPCLPointCloud2(cloud_filtered, *downsampled_XYZ);

    std::cerr << "PointCloud after filtering: " << downsampled_XYZ->width * downsampled_XYZ->height << " data points." << std::endl;
    ///@}


    ///@{   Crop region of interest
    Eigen::Vector4f minPoint; 
    minPoint[0]=0;  // define minimum point x 
    minPoint[1]=0;  // define minimum point y 
    minPoint[2]=0;  // define minimum point z 
    Eigen::Vector4f maxPoint; 
    maxPoint[0]=7;  // define max point x 
    maxPoint[1]=7;  // define max point y 
    maxPoint[2]=7;  // define max point z
// Define translation and rotation ( this is optional) 

    Eigen::Vector3f boxTranslatation; 
    boxTranslatation[0]=0.0;   
    boxTranslatation[1]=0.0;   
    boxTranslatation[2]=0.0;   
//    // this moves your cube from (0,0,0)//minPoint to (1,2,3)  // maxPoint is now(6,8,10) 


    Eigen::Vector3f boxRotation; 
    boxRotation[0]=0;  // rotation around x-axis 
    boxRotation[1]=0;  // rotation around y-axis 
    boxRotation[2]=0.0;  //in radians rotation around z-axis. this rotates your cube 45deg around z-axis. 

    // OR:
    // cropFilter.setTransform(boxTransform);
    
    // OR:
  // pass.setInputCloud (cloud);
  // pass.setFilterFieldName ("z");
  // pass.setFilterLimits (0, 2.8);
  // pass.filter (*cloud_filtered);
  // std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

    pcl::CropBox<pcl::PointXYZ> cropFilter; 
    cropFilter.setInputCloud (downsampled_XYZ); 
    cropFilter.setMin(minPoint); 
    cropFilter.setMax(maxPoint); 
    // cropFilter.setTranslation(boxTranslatation); 
    // cropFilter.setRotation(boxRotation); 

    cropFilter.filter (*downsampled_XYZ); 
    ///@}

    ///@{   Clustering
    //Create the SACSegmentation object and set the model and method type
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);//For more info: wikipedia.org/wiki/RANSAC
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);//How close a point must be to the model to considered an inlier

    int i = 0, nr_points = (int) downsampled_XYZ->points.size ();

    //Contains the plane point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    // While 30% of the original cloud is still there
    while (downsampled_XYZ->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (downsampled_XYZ);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (downsampled_XYZ);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        // std::cerr << "PointCloud representing the planar component: " 
        //         << output_p->width * output_p->height << " data points." << std::endl;

        // std::stringstream ss;
        // ss << "table_scene_lms400_plane_" << i << ".pcd";
        // writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        downsampled_XYZ.swap(cloud_f);
        i++;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (downsampled_XYZ);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (500);
    ec.setSearchMethod (tree);
    ec.setInputCloud (downsampled_XYZ);
    ec.extract (cluster_indices);

    //Create a publisher for each cluster
    ros::NodeHandle nh;
    for (int i = 0; i < cluster_indices.size(); ++i)
    {
        std::string topicName = "/pcl_tut/cluster" + boost::lexical_cast<std::string>(i);
        ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> (topicName, 1);
        g_pub_vec.push_back(pub);
    }

    int j = 0;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_clusters;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        cloud_clusters.push_back(cloud_cluster);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (downsampled_XYZ->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        
        //Convert the pointcloud to be used in ROS
        sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2);
        pcl::toROSMsg (*cloud_cluster, *output);
        output->header.frame_id = input->header.frame_id;

        // Publish the data
        g_pub_vec[j].publish (output);
        ++j;
    }
    ///@}

    ROS_INFO("Num of clusters: %zu", cloud_clusters.size());

    std::vector<Eigen::Vector4f> centroids;
    std::vector<std::pair<Eigen::Vector4f, int> > current_centroid_ids;
    visualization_msgs::MarkerArray ma;

    for (int i = 0; i < cloud_clusters.size(); ++i) {
        ///@{   Cylinder fitting
        pcl::PassThrough<pcl::PointXYZ> pass;
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg2; 
        pcl::PCDWriter writer;
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ> ());
      
        // Datasets
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
        pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
      
        std::cerr << "PointCloud has: " << cloud_clusters[i]->points.size () << " data points." << std::endl;
      
        // Build a passthrough filter to remove spurious NaNs
        pass.setInputCloud (cloud_clusters[i]);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0, 1.5);
        pass.filter (*cloud_filtered2);
        std::cerr << "PointCloud after filtering has: " << cloud_filtered2->points.size () << " data points." << std::endl;
      
        // Estimate point normals
        ne.setSearchMethod (tree2);
        ne.setInputCloud (cloud_filtered2);
        ne.setKSearch (50);
        ne.compute (*cloud_normals2);
      
        // Create the segmentation object for cylinder segmentation and set all the parameters
        seg2.setOptimizeCoefficients (true);
        seg2.setModelType (pcl::SACMODEL_CYLINDER);
        seg2.setMethodType (pcl::SAC_RANSAC);
        seg2.setNormalDistanceWeight (0.1);
        seg2.setMaxIterations (10000);
        seg2.setDistanceThreshold (0.05);
        seg2.setRadiusLimits (0.02, 0.1);
        seg2.setInputCloud (cloud_filtered2);
        seg2.setInputNormals (cloud_normals2);
      
        // Obtain the cylinder inliers and coefficients
        seg2.segment (*inliers_cylinder, *coefficients_cylinder);
        // std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
    
        // Write the cylinder inliers to disk
        extract.setInputCloud (cloud_filtered2);
        extract.setIndices (inliers_cylinder);
        extract.setNegative (false);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZ> ());
        extract.filter (*cloud_cylinder);
        sensor_msgs::PointCloud2::Ptr output_cyl (new sensor_msgs::PointCloud2);
      
        //color
        // pcl::PointCloud<pcl::PointXYZRGB> cloud_cylinder_color;
        // pcl::copyPointCloud(*cloud_cylinder, cloud_cylinder_color);
        // uint8_t r = 0, g = 255, b = 0;    // Example: Red color
        // uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
      
        // for (size_t i = i; i < cloud_cylinder->points.size(); ++i) {
        //     // output_cyl->points[i].r = 200;
        //     cloud_cylinder_color.points[i].rgb = *reinterpret_cast<float*>(&rgb);
        // }
      
        #if 0 // publish cylinder inlier cloud
        pcl::toROSMsg (*cloud_cylinder, *output_cyl);

        output_cyl->header.frame_id = input->header.frame_id;
        g_pub_cyl_cloud.publish(output_cyl);
        #endif


        if (cloud_cylinder->points.empty ()) {
          std::cerr << "Can't find the cylindrical component." << std::endl;
          continue;
        }
        else
        {
          std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
          // writer.write ("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
        }
        // ROS_INFO("Cloud in: %zu, Cloud out: %zu", cloud_filtered2->points.size(), cloud_cylinder->points.size ());
        // if (cloud_cylinder->points.size() * 1.0/ cloud_filtered2->points.size() < 0.5) {
        //     ROS_INFO("Very few inliers so ignoring");
        //     continue;
        // }
        ///@}

        ///@{   Centroids
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_cylinder, centroid);
        centroids.push_back(centroid);
      
        ROS_INFO("Centroid x: %f  y: %f  z: %f\n", centroid[0], centroid[1], centroid[2]);
    
        ///@{   Cylinder ids
        double min_dist = 1000000;
        double thresh = 0.1;
        int centroid_id;

        for (size_t cdx = 0; cdx < g_last_centroids.size(); ++cdx) {
            Eigen::Vector4f c = g_last_centroids[cdx].first;
            double dist = sqrt(pow(c[0] - centroid[0], 2) + pow(c[0] - centroid[0], 2) + pow(c[0] - centroid[0], 2));
            if (dist < min_dist) {
                min_dist = dist;
                centroid_id = g_last_centroids[cdx].second;
            }
        }
        ROS_INFO("dist: %f", min_dist);
        if (min_dist > thresh) {
            centroid_id = g_last_centroids.size();
            g_last_centroids.push_back(std::pair<Eigen::Vector4f, int> (centroid, centroid_id));
        }

        current_centroid_ids.push_back(std::pair<Eigen::Vector4f, int> (centroid, centroid_id));
        ///@}

        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "camera_depth_optical_frame";
        marker.ns = "cylinder";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = centroid[0];
        marker.pose.position.y = centroid[1];
        marker.pose.position.z = centroid[2];
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.1;
        if (centroid_id == 1) {
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
        }
        else if (centroid_id == 0) {
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;          
        }
        else{
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;          
        } 
        marker.color.a = 1.0f;
        marker.lifetime = ros::Duration(0);
        ma.markers.push_back(marker);
    }

    // update centroids
    g_last_centroids = current_centroid_ids;

    ROS_INFO("\n %zu CENTROID IDS: ", g_last_centroids.size());
    for (int i = 0; i < g_last_centroids.size(); ++i) {
      ROS_INFO("    id: %d", g_last_centroids[i].second);
    }

    // getchar();

    g_pub_cyl_markers.publish(ma);
    ///@}
  
    ros::Time end = ros::Time::now();
    ros::Duration d = end - begin;
  
    ROS_INFO("Cycle time %f", d.toSec());
    return;
}
int
main (int argc, char** argv)
{
    ros::init (argc, argv, "cluster_extraction");
    ros::NodeHandle nh;

    g_pub_cyl_cloud = nh.advertise<sensor_msgs::PointCloud2> ("cyl_cloud", 100);
    g_pub_cyl_markers = nh.advertise<visualization_msgs::MarkerArray>(
            "visualization_markers", 100);
    // Initialize ROS

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);
    

    // Create a ROS publisher for the output point cloud
    // pub = nh.advertise<sensor_msgs::PointCloud2> ("/pcl_tut/object_cluster", 1);

    // Spin
    ros::spin ();
}