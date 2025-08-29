#include <ros/ros.h>
#include <cmath>
#include <fstream>
#include <csignal>
#include <random>

/*PCL*/
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/segmentation/grabcut_segmentation.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/search/organized.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/don.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/angles.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/filters/radius_outlier_removal.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include <pcl/visualization/cloud_viewer.h>

#pragma GCC diagnostic pop


#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <laser_geometry/laser_geometry.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include "mapping_msgs/BoundingBoxes.h"
#include "mapping_msgs/SemanticMap.h"
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <pcl/filters/model_outlier_removal.h>


#include "boxsyncpolicy.h"
#include "semanticmap.h"
#include "boost_geometry_msgs.h"

#include <dynamic_reconfigure/server.h>
#include <semmapping/ParamsConfig.h>
#include "point_cloud_color_handler_clusters.h"

#include <chrono>
#include <mapping_msgs/BoxesAndClouds.h>
#include <mapping_msgs/FindObjects.h>
#include <mapping_msgs/ObjectPositions.h>
//#include <mapping_msgs/ObbMap.h>
#include "rosgraph_msgs/Clock.h"
#include <pcl/sample_consensus/sac_model_normal_plane.h>


#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
namespace bg = boost::geometry;
using semmapping::polygon;
using semmapping::point;

//#define SHOW_POINTCLOUD_VISUALIZATION

tf2_ros::Buffer tfBuffer(ros::Duration(20));
ros::Publisher objectPgPub;
ros::Publisher detectedPgPub;
ros::Publisher camPgPub;
ros::Publisher laserPgPub;
ros::Publisher observationPgPub;
ros::Publisher completeAreaPgPub;
ros::Publisher semanticMapPub;
ros::Publisher gtSemanticMapPub;
ros::Publisher debugCloudPub;
ros::Publisher debug2CloudPub;
ros::Publisher position_pub;
ros::Publisher objectMarkerPub;
ros::Publisher objectCloudPub;


//Testing files paths
//std::string ground_truth_map_file_name= "src/sem_mapping/semmapping/maps/truth_map_well_arranged_world.yaml";
std::string ground_truth_map_file_name= "src/sem_mapping/semmapping/maps/truth_map_cluttered_world_2.yaml";

//pcl::visualization::CloudViewer *viewer;
pcl::visualization::PCLVisualizer *viewer;
boost::mutex viewer_mtx;
int vport0 = 0;
int vport1 = 1;
int vport2 = 2;
int vport3 = 3;
int semmap_vport0 = 4;
int semmap_vport1 = 5;
semmapping::ParamsConfig param_config;
semmapping::SemanticMap map(tfBuffer, viewer, viewer_mtx, semmap_vport0, semmap_vport1, param_config);


int viewer_index = 0;
bool static_map;

std::list<std::string> consideredCategories{"Chair","Table","Couch","Shelf", "Bed","Trash can"};

// Function to add Gaussian noise to a sensor_msgs::PointCloud2 message
sensor_msgs::PointCloud2::Ptr addNoiseToPointCloud(const sensor_msgs::PointCloud2::ConstPtr& input_cloud)
{
    // Convert sensor_msgs::PointCloud2 to PCL point c.txt"loud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_cloud, *pcl_cloud);

    // Parameters for noise addition (you can adjust these as needed)
    double mean = 0.0;
    double stddev = 0.02; // standard deviation of the noise

    // Add Gaussian noise to each point
    for (size_t i = 0; i < pcl_cloud->points.size(); ++i)
    {
        pcl_cloud->points[i].x += static_cast<float>(mean + stddev * std::rand() / (RAND_MAX + 1.0));
        pcl_cloud->points[i].y += static_cast<float>(mean + stddev * std::rand() / (RAND_MAX + 1.0));
        pcl_cloud->points[i].z += static_cast<float>(mean + stddev * std::rand() / (RAND_MAX + 1.0));
    }

    // Convert back to sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2::Ptr noisy_cloud(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*pcl_cloud, *noisy_cloud);

    // Set the same header as the input cloud
    noisy_cloud->header = input_cloud->header;

    return noisy_cloud;
}

inline bool operator==(const geometry_msgs::Vector3 &lhs, const geometry_msgs::Vector3 &rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

inline bool operator==(const geometry_msgs::Quaternion &lhs, const geometry_msgs::Quaternion &rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z && lhs.w == rhs.w;
}

inline bool operator==(const geometry_msgs::Transform &lhs, const geometry_msgs::Transform &rhs) {
    return lhs.translation == rhs.translation && lhs.rotation == rhs.rotation;
}

bool inConsideredObjectList(std::string object_name){
    bool found = (std::find(consideredCategories.begin(), consideredCategories.end(), object_name) != consideredCategories.end());
    return found;
}

void visualizeCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, double vport) {
    viewer_mtx.lock();
    viewer->removeAllPointClouds(vport);
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud" + std::to_string(viewer_index + vport), vport);
    viewer_mtx.unlock();
}

void visualizeColorCustomCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, double vport, int id) {
    viewer_mtx.lock();
    viewer->removeAllPointClouds(vport);
    viewer->setBackgroundColor(0, 0, 0);
    pcl::RGB color = pcl::GlasbeyLUT::at(id);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> roi_color_handler(cloud, color.r, color.g, color.b);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, roi_color_handler, "cloud" + std::to_string(viewer_index + vport),
                                         vport);
    viewer_mtx.unlock();
}

void visualizeColorCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const std::vector<pcl::PointIndices> &clusters,
                         double vport) {
    viewer_mtx.lock();
    viewer->removeAllPointClouds(vport);
    viewer->setBackgroundColor(0, 0, 0);

    pcl::visualization::PointCloudColorHandlerClusters<pcl::PointXYZ> roi_color_handler(cloud, clusters);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, roi_color_handler, "cloud" + std::to_string(viewer_index + vport),
                                         vport);
    viewer_mtx.unlock();
}

/*std::pair<double, double> get_real_object_length_width(const std::string &name)
    {
        std::pair<double, double> dimensions;

        //if(name=="Chair") {dimensions.first= 0.6; dimensions.second= 0.57;} // Chair model in world well arranged
        if(name=="Chair") {dimensions.first= 0.57; dimensions.second= 0.57;}  // Chair model in world cluttered
        else if (name== "Table") {dimensions.first= 1.782; dimensions.second= 0.8;}
        else if (name=="Shelf") {dimensions.first= 0.9; dimensions.second= 0.4;}
        else if (name=="Couch"){dimensions.first= 0.97; dimensions.second= 2.009;}
        else if (name=="Man"){dimensions.first= 0.6; dimensions.second= 0.3;}
        else {dimensions.first= 0.0; dimensions.second= 0.0;}

        return dimensions;
    }*/
    std::vector<std::pair<double, double>> get_object_contour(const std::string& name)
    {
        std::vector<std::pair<double, double>> contour;
        std::string path = "/home/ismael/basedeconnaissance/";
        std::string filename;

        if (name == "Chair") filename = "chaise.txt";
        else if (name == "Table") filename = "table.txt";
        else if (name == "Shelf") filename = "shelf.txt";
        else if (name == "Couch") filename = "couch.txt";
        else if (name == "Bed") filename = "bed.txt";
        else if (name == "Trash can") filename = "Trash can.txt";
        else {
            ROS_WARN("Aucun contour connu pour l'objet : %s", name.c_str());
            return contour;  // contour vide
        }

        std::ifstream infile(path + filename);
        if (!infile.is_open()) {
            ROS_WARN("Impossible d'ouvrir le fichier de contour : %s", (path + filename).c_str());
            return contour;
        }

        double x, y;
        while (infile >> x >> y) {
            contour.emplace_back(x, y);
        }

        ROS_INFO("Contour chargé pour l'objet '%s' (%lu sommets                                                                         )", name.c_str(), contour.size());
        return contour;
    }

semmapping::polygon getPolygonInMap(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    if (cloud->size() == 0) {
        ROS_WARN("Point cloud was empty, could not compute area");
        return semmapping::polygon();
    }
    // Create a set of planar coefficients with X=Y=0,Z=1 (XY-plane)
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;

    // Project the cloud in the XY-plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setModelCoefficients(coefficients);
    proj.setInputCloud(cloud);
    proj.filter(*cloud_projected);

    // Filter the projected points
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_projected);
    sor.setLeafSize(0.02f, 0.02f, 0.02f);
    sor.filter(*cloud_filtered);


    // Create a Concave Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud(cloud_filtered);
    chull.setDimension(2);
    chull.reconstruct(*cloud_hull);
      

    


    return semmapping::pclToBoost(*cloud_hull);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudFromInd(std::vector<pcl::PointIndices> input_indices,
                                                      pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud) {
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (pcl::PointIndices indi : input_indices) {
        pcl::PointIndices::Ptr tmp_ind(new pcl::PointIndices(indi));
        extract.setInputCloud(input_cloud);
        extract.setIndices(tmp_ind);
        extract.filter(*tmp_cloud);
        *result_cloud += *tmp_cloud;

    }
    return result_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudFromIndSingle(pcl::PointIndices::Ptr input_indices, pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud) {
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(input_cloud);
    extract.setIndices(input_indices);
    extract.filter(*result_cloud);
    return result_cloud;
}

std::vector<pcl::PointIndices> getObjectPointsEuc(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointIndices::ConstPtr input_indicies) {
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    std::vector<pcl::PointIndices> cluster_indices;

    if (input_indicies->indices.size() == 0) {
        ROS_WARN("No indices in cloud before Euclead");
        return cluster_indices;
    }

    if (cloud->points.size() == 0) {
        ROS_WARN("No points in cloud before Euclead and tree");
        return cluster_indices;
    }
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    //Do eucledean clustering
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(param_config.euc_tolerance);
    ec.setMinClusterSize(50);//param_config.euc_min_cluster
    ec.setMaxClusterSize(cloud->points.size());
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.setIndices(input_indicies);
    ec.extract(cluster_indices);
    if (cluster_indices.size() == 0) {
        ROS_WARN("No points in cluster indicies");
        return cluster_indices;
    }
    return cluster_indices;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr removeRadiusOutliers(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // Create the filtering object
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.02);
    outrem.setMinNeighborsInRadius(50);
    outrem.setKeepOrganized(true);
    // apply filter
    outrem.filter(*cloud_filtered);
    return cloud_filtered;
}

pcl::PointIndices::Ptr
removeOutliers(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointIndices::ConstPtr input_indis) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // Create the filtering object
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(100);
    sor.setStddevMulThresh(0.5);
    sor.setIndices(input_indis);
    std::vector<int> indis;
    sor.filter(indis);
    inliers->indices = indis;
    return inliers;
}


pcl::PointIndices::Ptr
filterPlanes(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointIndices::ConstPtr input_indices, bool isTable) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    if (isTable)
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    else
        seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    if (isTable)
        seg.setDistanceThreshold(0.01);
    else
        seg.setDistanceThreshold(0.03);
    seg.setInputCloud(cloud);
    seg.setIndices(input_indices);
    seg.setMaxIterations(2000);
    Eigen::Vector3f axis;
    axis << 0, 0, 1;
    seg.setAxis(axis);
    seg.setEpsAngle(0.261799); //15

    seg.segment(*inliers, *coefficients);
    return inliers;
}

double calculateMeanHight(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud) {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*input_cloud, centroid);
    return centroid[2];
}

pcl::PointIndices::Ptr dsBoxPoints(int ymin, int xmin, int height, int width, pcl::PointCloud<pcl::PointXYZ>::ConstPtr orig_cloud,
            pcl::VoxelGrid<pcl::PointXYZ> &vox) {
    pcl::PointIndices::Ptr inds(new pcl::PointIndices);
    std::set<int> ind_set;
    for (int y = ymin; y < ymin + height; y++) {
        for (int x = xmin; x < xmin + width; x++) {
            const pcl::PointXYZ &p = orig_cloud->at(x, y);
            Eigen::Vector3i coords = vox.getGridCoordinates(p.x, p.y, p.z);
            int index = vox.getCentroidIndexAt(coords);
            if (index >= 0)
                ind_set.insert(index);
        }
    }
    for (int index : ind_set)
        inds->indices.push_back(index);
    return inds;
}


std::vector<pcl::PointIndices> floorDetection(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud) {
    std::vector<pcl::PointIndices> return_indices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr floor_inds(new pcl::PointIndices);
    pcl::PointIndices::Ptr without_floor_inds(new pcl::PointIndices);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(input_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.2, 0.15);
    pass.setNegative(true);
    pass.filter(without_floor_inds->indices);
    pass.setNegative(false);
    pass.filter(floor_inds->indices);
    return_indices.push_back(*without_floor_inds);
    return_indices.push_back(*floor_inds);
    return return_indices;
}

void indiceBoxAssociaton(const std::vector<pcl::PointIndices> &clusters, pcl::PointIndices::Ptr &box_inds,
                         std::vector<pcl::PointIndices> &best_inds, bool isTable, std::string Class) {
    int max_ind_size = 0;
    for (pcl::PointIndices indi : clusters) {
        pcl::PointIndices::Ptr temp_inds(new pcl::PointIndices);
        std::set_intersection(box_inds->indices.begin(), box_inds->indices.end(), indi.indices.begin(),
                              indi.indices.end(), std::back_inserter(temp_inds->indices));
        if (temp_inds != nullptr) {
            if (temp_inds->indices.size() > max_ind_size) {
                max_ind_size = temp_inds->indices.size();
                best_inds.clear();
                best_inds.push_back(*temp_inds);
            }
        }
    }
    if (best_inds.size() == 0) {
        best_inds.push_back(*box_inds);
        ROS_WARN_STREAM("best ind size = 0 for obj: " << Class);
    }
}

void mapUpdateAndPublish(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclCloud, semmapping::point &robot_position, sensor_msgs::PointCloud2::Ptr cloud)//const sensor_msgs::PointCloud2::Ptr &cloud, const mapping_msgs::BoundingBoxes::ConstPtr &boxes)
{
    semmapping::polygon comp_area = getPolygonInMap(pclCloud);
    if (comp_area.outer().size() > 0)
    {
        geometry_msgs::PolygonStamped comp_area_msg;
        comp_area_msg.polygon = semmapping::boostToPolygonMsg(comp_area);
        comp_area_msg.header.frame_id = "map";
        comp_area_msg.header.stamp = cloud->header.stamp;
        completeAreaPgPub.publish(comp_area_msg);
        map.removeEvidence(comp_area, robot_position);
    }
    mapping_msgs::SemanticMap::Ptr map_msg = map.createMapMessage(robot_position, false, debugCloudPub);
    semanticMapPub.publish(map_msg);
    return;
}

void processBoxes(const mapping_msgs::BoxesAndClouds &data){   
    if (static_map == false) {
        const sensor_msgs::PointCloud2::Ptr input_cloud(new sensor_msgs::PointCloud2(data.point_cloud));
        
        sensor_msgs::PointCloud2::Ptr cloud = addNoiseToPointCloud(input_cloud);

        const mapping_msgs::BoundingBoxes::ConstPtr boxes(new mapping_msgs::BoundingBoxes(data.bounding_boxes));

        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped transform;
        try {
            transform = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
        }
        catch (tf2::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        semmapping::point robot_position;
        robot_position.x(transform.transform.translation.x);
        robot_position.y(transform.transform.translation.y);

        // check if robot has moved, if not, do nothing
        // Static variable to store the last known position of the robot

        semmapping::point last_robot_position;
        static bool is_first_run = true;

        if (!is_first_run) {
            // Calculate the distance between the current and last robot positions
            double position_error = sqrt(pow(robot_position.x() - last_robot_position.x(), 2) +
                                         pow(robot_position.y() - last_robot_position.y(), 2));

            // Check if the position error is less than 1 mm (0.001 meters)
            if (position_error < 0.001) {
                // ROS_INFO("Robot has not moved significantly, position error: %f meters", position_error);
                return;
            }
        }

        // Update the last known position
        last_robot_position = robot_position;
        is_first_run = false; 


        // Get camera position and transform it to Eigen
        geometry_msgs::TransformStamped camPose;
        try {
            camPose = tfBuffer.lookupTransform("map", cloud->header.frame_id, cloud->header.stamp, ros::Duration(0));
        }
        catch (tf2::TransformException ex) {
            ROS_WARN_STREAM("Could not read transform: " << ex.what());
            return;
        }
        Eigen::Affine3d camPoseEigen = tf2::transformToEigen(camPose.transform);

        // Transform the point cloud to a PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud, *pclCloud);

        if (pclCloud->empty()) {
            ROS_WARN("Point cloud is empty, not processing boxes");
            return;
        }

        pcl::transformPointCloud(*pclCloud, *pclCloud, camPoseEigen);

        if (boxes->bounding_boxes.size() == 0) {
            ROS_WARN_STREAM("DETECTION BOX IS EMPTY");
            mapping_msgs::SemanticMap::Ptr map_msg;
            mapUpdateAndPublish(pclCloud, robot_position, cloud);
            return;
        }

        /*Filtering the point cloud: downsample the original point clouds using a leaf size of 2cm*/
        pcl::PointCloud<pcl::PointXYZ>::Ptr ds_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(pclCloud);
        sor.setLeafSize(0.02, 0.02, 0.02);
        sor.setSaveLeafLayout(true);
        sor.filter(*ds_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr orig_cloud = pclCloud;
        pclCloud = ds_cloud;

        /*Identifying and deleting floor, wall, table points from the point cloud*/
        pcl::PointIndices::Ptr floor_inds(new pcl::PointIndices);
        pcl::PointIndices::Ptr without_floor_inds(new pcl::PointIndices);
        std::vector<pcl::PointIndices> floor_ind_vector;

        floor_ind_vector = floorDetection(pclCloud);
        without_floor_inds->indices = floor_ind_vector[0].indices;
        floor_inds->indices = floor_ind_vector[1].indices;


        pcl::PointIndices::Ptr wall_inds(new pcl::PointIndices);
        wall_inds = filterPlanes(pclCloud, without_floor_inds, false);

        pcl::PointIndices::Ptr inds_without_floor_and_walls(new pcl::PointIndices);
        std::set_difference(without_floor_inds->indices.begin(), without_floor_inds->indices.end(),
                            wall_inds->indices.begin(), wall_inds->indices.end(),
                            std::back_inserter(inds_without_floor_and_walls->indices));

        pcl::PointIndices::Ptr table_inds(new pcl::PointIndices);
        auto it = std::find_if(boxes->bounding_boxes.begin(), boxes->bounding_boxes.end(),
                               [](const mapping_msgs::BoundingBox &box) { return box.Class == "Table"; });
        if (it != boxes->bounding_boxes.end()) {// Table found
            table_inds = filterPlanes(pclCloud, inds_without_floor_and_walls, true);
        }


        pcl::PointIndices::Ptr inds_without_floor_wall_and_table(new pcl::PointIndices);
        std::set_difference(inds_without_floor_and_walls->indices.begin(), inds_without_floor_and_walls->indices.end(),
                            table_inds->indices.begin(), table_inds->indices.end(),
                            std::back_inserter(inds_without_floor_wall_and_table->indices));

        pcl::PointIndices::Ptr inds_outlier_free(new pcl::PointIndices);
        inds_outlier_free = removeOutliers(pclCloud, inds_without_floor_wall_and_table);

        /*Clustering object point clouds*/
        std::vector<pcl::PointIndices> debug_vis_box_clusters;
        std::vector<pcl::PointIndices> clusters;
        std::vector<pcl::PointIndices> table_clusters;
        clusters = getObjectPointsEuc(pclCloud, inds_outlier_free);

        /*Saving tables clusters*/
        if (table_inds->indices.empty() == false)
            table_clusters = getObjectPointsEuc(pclCloud, table_inds);

        for (const mapping_msgs::BoundingBox &box : boxes->bounding_boxes) {
            ROS_INFO_STREAM("Traitement de la boîte : " << box.Class << ", probabilité : " << box.probability);
            std::vector<pcl::PointIndices> best_inds;
            pcl::PointIndices::Ptr box_inds(new pcl::PointIndices);
            box_inds = dsBoxPoints(box.ymin, box.xmin, box.ymax - box.ymin, box.xmax - box.xmin, orig_cloud, sor);

            std::string Class = box.Class;

            // Consider Sofa bed and Couch as similar classes
            if (Class == "Sofa bed") 
                Class = "Couch" ;

            if(!inConsideredObjectList(Class)){
                ROS_WARN_STREAM("Object "<< Class<<" is not in considered object list.");
                continue;
            }

            /*Associate clusters to detection boxes*/
            if (Class == "Table") {
                if (table_clusters.size() > 0)
                    indiceBoxAssociaton(table_clusters, box_inds, best_inds, true, Class);
                else indiceBoxAssociaton(clusters, box_inds, best_inds, false, Class);
            } else {
                indiceBoxAssociaton(clusters, box_inds, best_inds, false, Class);
            }

            /*Save object 3D model in res_cloud*/
            pcl::PointCloud<pcl::PointXYZ>::Ptr res_cloud(new pcl::PointCloud<pcl::PointXYZ>);

            if (best_inds.size() > 0)
                res_cloud = pointCloudFromInd(best_inds, pclCloud);
            else {
                ROS_WARN_STREAM("best ind size = 0 for obj: " << Class);
                continue;
            }
                        // Enregistrement du nuage de points 3D au format TXT
            std::string filename = "/home/ismael/pointclouds/" + Class + "_" + std::to_string(ros::Time::now().toNSec()) + ".txt";

            std::ofstream file(filename);
            if (file.is_open()) {
                for (const auto& point : res_cloud->points) {
                    file << point.x << " " << point.y << " " << point.z << "\n";
                }
                file.close();
                ROS_INFO_STREAM("Saved 3D point cloud to: " << filename);
            } else {
                ROS_WARN_STREAM("Failed to save point cloud for object: " << Class);
            }
            //  Ajouter cette partie pour publier dans RViz
            sensor_msgs::PointCloud2 cloud_msg;
            pcl::toROSMsg(*res_cloud, cloud_msg);
            cloud_msg.header.frame_id = "map";  //  ou "base_link" si c'est ton frame principal
            cloud_msg.header.stamp = ros::Time::now();
            objectCloudPub.publish(cloud_msg);
            ROS_INFO_STREAM("Published point cloud for object: " << Class << " with " << res_cloud->size() << " points");


            /*Create the polygon convex from the 3D model (res_cloud) and save it in res_pg*/
            semmapping::polygon res_pg = getPolygonInMap(res_cloud);

            if (res_pg.outer().empty()) {
                ROS_WARN_STREAM("Polygon in map could not be reconstructed for obj: " << Class);
                continue;
            }

            /*Verify that the polygon is in robot good perception zone*/
            semmapping::point res_pg_cen;
            semmapping::bg::centroid(res_pg, res_pg_cen);
            double dx = res_pg_cen.x() - robot_position.x();
            double dy = res_pg_cen.y() - robot_position.y();
            double distance_from_robot = sqrt(dx * dx + dy * dy);

            ROS_INFO_STREAM("Distance de l’objet " << Class << " au robot : " << distance_from_robot);

            if (distance_from_robot < 1.7 || distance_from_robot > 3.0) {
                std::string warning_message = Class + " object is not in the robot good perception zone.";
                ROS_WARN_STREAM(warning_message);

                continue;
            }
            /*
            //Verify that the polygon area is good enough to be added to the map ( >= 0.3*realArea and <= realArea). //
            std::pair<double, double> dimensions = get_real_object_length_width(Class);
            double realArea = dimensions.first * dimensions.second;

            double polygonArea = semmapping::bg::area(res_pg);
            ROS_INFO_STREAM("Aire du polygone : " << polygonArea << ", Aire réelle attendue pour " << Class << " : " << realArea);
            if (polygonArea >= 0.05 * realArea && polygonArea <= realArea) {      
                map.addEvidence(Class, box.probability, res_pg, calculateMeanHight(res_cloud), res_cloud);
            } else {
                map.updateEvidenceCertainty(Class, res_pg);
                std::string warning_message = "Object " + Class + " area is not good enough to update object shape.";
                ROS_WARN_STREAM(warning_message);
            }*/
            // Récupérer le contour réel (vecteur de points)
            std::vector<std::pair<double, double>> ref_pts = get_object_contour(Class);

            // Vérifier que la base contient bien un contour
            if (ref_pts.empty()) {
                ROS_WARN_STREAM("Contour réel non trouvé pour la classe : " << Class);
                continue;
            }

            // Convertir en polygon Boost.Geometry
            polygon ref_contour;
            for (const auto& p : ref_pts) {
                bg::append(ref_contour.outer(), point(p.first, p.second));
            }

            // Fermer le contour si nécessaire
            if (ref_pts.front() != ref_pts.back()) {
                bg::append(ref_contour.outer(), point(ref_pts.front().first, ref_pts.front().second));
            }
            // ✅ Corriger l’orientation du contour pour éviter les aires négatives
            bg::correct(ref_contour);

            // Calculer les aires
            double realArea = bg::area(ref_contour);
            double polygonArea = semmapping::bg::area(res_pg);

            ROS_INFO_STREAM("Aire du polygone : " << polygonArea << ", Aire réelle attendue pour " << Class << " : " << realArea);

            // Vérification par seuils (adapter si nécessaire)
            if (polygonArea >= 0.3* realArea && polygonArea <= realArea) {
                map.addEvidence(Class, box.probability, res_pg, calculateMeanHight(res_cloud), res_cloud);
            } else {
                map.updateEvidenceCertainty(Class, res_pg);
                std::string warning_message = "Object " + Class + " area is not good enough to update object shape.";
                ROS_WARN_STREAM(warning_message);
            }
            
        }
        mapUpdateAndPublish(orig_cloud, robot_position, cloud);
    }
}


std::string readNext(const std::string &str, std::string::const_iterator &begin) {
    std::string::const_iterator end = str.cend();
    std::string res;
    res.reserve(str.size());

    for (; begin != end && *begin == ' '; begin++); // skip leading whitespaces

    char end_char = ' ';
    if (begin != end && *begin == '"') {
        end_char = '"';
        begin++;
    }

    bool escape = false;
    for (; begin != end; begin++) {
        if (escape)
            escape = false;
        else if (*begin == '\\') {
            escape = true;
            continue;
        } else if (*begin == end_char) {
            begin++; // remove end character
            break;
        }
        res += *begin;
    }
    return res;
}

void sigintHandler(int sig) {
    exit(0);
}

void callback(semmapping::ParamsConfig &config, uint32_t level) {
    param_config = config;
}

void findObject(const mapping_msgs::FindObjects &request)
{
    mapping_msgs::ObjectPositions::Ptr position_msg = map.findObjectPosition(request);
    position_pub.publish(position_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mapping");
    ros::NodeHandle nh("~");

    dynamic_reconfigure::Server<semmapping::ParamsConfig> server;

    server.setCallback(callback);

    tf2_ros::TransformListener tfListener(tfBuffer);

    std::string detector = nh.param<std::string>("detector", "detector_node");

    //hsr
    std::string point_cloud_topic = nh.param<std::string>("point_cloud", "/camera/depth/points");
    std::string camera_info_topic = nh.param<std::string>("camera_info", "/camera/rgb/camera_info");
    std::string laser_scanner_topic = nh.param<std::string>("laser_scanner", "/scan");

    ros::Subscriber boundingBoxSub = nh.subscribe("/cloud_and_boxes", 10, &processBoxes);
    ros::Subscriber finderSub = nh.subscribe("/find_object", 10, &findObject);

    /*Initialize publisher*/
    objectPgPub = nh.advertise<geometry_msgs::PolygonStamped>("object_pg", 1, true);
    detectedPgPub = nh.advertise<geometry_msgs::PolygonStamped>("detected_pg", 1, true);
    camPgPub = nh.advertise<geometry_msgs::PolygonStamped>("camera_pg", 1, true);
    laserPgPub = nh.advertise<geometry_msgs::PolygonStamped>("laser_pg", 1, true);
    observationPgPub = nh.advertise<geometry_msgs::PolygonStamped>("observation_pg", 1, true);
    completeAreaPgPub = nh.advertise<geometry_msgs::PolygonStamped>("complete_area_pg", 1, true);
    semanticMapPub = nh.advertise<mapping_msgs::SemanticMap>("/semantic_map", 1, true);
    gtSemanticMapPub = nh.advertise<mapping_msgs::SemanticMap>("/ground_truth_semantic_map", 1, true);
    debugCloudPub = nh.advertise<sensor_msgs::PointCloud2>("debug_cloud", 1, true);
    debug2CloudPub = nh.advertise<sensor_msgs::PointCloud2>("debug2_cloud", 1, true);
    position_pub = nh.advertise<mapping_msgs::ObjectPositions>("found_objects", 1, true);
    objectMarkerPub = nh.advertise<visualization_msgs::Marker>("object_markers", 1, true);
    objectCloudPub = nh.advertise<sensor_msgs::PointCloud2>("object_clouds", 1, true);

    ros::AsyncSpinner spinner(16); //16
    spinner.start();

    /*Load saved Map*/
    static_map = nh.param<bool>("static_map", false);
    bool load_file = nh.param<bool>("load_file", false);
    if (load_file) {
        if (!nh.hasParam("file")) {
            ROS_ERROR("File to load not specified");
            return -1;
        }
        std::string file_name;
        nh.getParam("file", file_name);
        std::ifstream file(file_name);
        if (!file) {
            ROS_ERROR("File could not be opened");
        } else {
            if (map.readMapData(file)) {
                semmapping::point robot;
                ROS_INFO("Map loaded successfully yeay");
                mapping_msgs::SemanticMap::Ptr map_msg = map.createMapMessage(robot, true, debugCloudPub);
                semanticMapPub.publish(map_msg);
            } else {
                ROS_ERROR("Failed loading map");
            }
            file.close();
        }
    }

    signal(SIGINT, sigintHandler);

    /*Terminal input loop*/
    std::cout << "Waiting for input. type \"help\" for help." << std::endl;
    while (1) {
        cout << "HALLO" << endl;
        std::string in;
        std::getline(std::cin, in);
        std::string::const_iterator it = in.cbegin();
        std::string command = readNext(in, it);
        if (command == "help") {
            std::cout << "Available commands:" << std::endl; // TODO
        } 
        else if (command == "load") {
            std::string fname = readNext(in, it);
            std::cout << "Loading map file: " << fname << std::endl;
            std::ifstream file(fname);
            if (!file) {
                std::cout << "File could not be opened" << std::endl;
                continue;
            }
            if (map.readMapData(file)) {
                semmapping::point robot;
                mapping_msgs::SemanticMap::Ptr map_msg= map.createMapMessage(robot, true, debugCloudPub);
                semanticMapPub.publish(map_msg);
                std::cout << "Map loaded successfully" << std::endl;
            } else {
                std::cout << "Failed loading map" << std::endl;
            }
            file.close();
        }
        
        else if (command == "load_ground_truth_map") {
            std::string ground_truth_map_file_name = "/home/ismael/smapping/src/sem_mapping/semmapping/maps/exp1_world.yaml";
            std::cout << "Loading ground truth map file: " << ground_truth_map_file_name << std::endl;
            std::ifstream file(ground_truth_map_file_name);
        
            if (!file) {
                std::cout << "Ground truth map could not be opened" << std::endl;
                continue;
            }
            if (map.loadGroundTruthMap(file)) {
                mapping_msgs::SemanticMap::Ptr map_msg= map.createGroundTruthMapMessage();
                gtSemanticMapPub.publish(map_msg);
                std::cout << "Ground truth Map loaded successfully" << std::endl;
            } else {
                std::cout << "Failed loading map" << std::endl;
            }
            file.close();
        } 
        else if (command == "save_likelihood") {
            std::string fname = readNext(in, it);
            std::cout << "Saving map file: " << fname << std::endl;
            std::ofstream file(fname);
            if (!file) {
                std::cout << "File could not be opened" << std::endl;
                continue;
            }
            if (map.writeLikelihoodData(file)) {
                std::cout << "Map saving successfully" << std::endl;
            } else {
                std::cout << "Failed saving map" << std::endl;
            }
            file.close();
        } 
        else if(command == "map_score"){
            //std::ifstream file(ground_truth_map_file_name);
            std::ifstream file("/home/ismael/smapping/src/sem_mapping/semmapping/maps/exp1_world.yaml");
            map.loadGroundTruthMap(file);
            mapping_msgs::SemanticMap::Ptr gt_map_msg= map.createGroundTruthMapMessage();
            gtSemanticMapPub.publish(gt_map_msg);
            std::ifstream file2("/home/ismael/map1.yaml");
            map.readMapData(file2);
            map.evaluteMap("/home/ismael/basedeconnaissance/resultat/world_tests.txt");
        }
        else if(command == "current_map_score"){
            std::ifstream file(ground_truth_map_file_name);
            //std::ifstream file("src/sem_mapping/semmapping/maps/truth_map_cluttered_world_2.yaml");
            map.loadGroundTruthMap(file);
            mapping_msgs::SemanticMap::Ptr gt_map_msg= map.createGroundTruthMapMessage();
            gtSemanticMapPub.publish(gt_map_msg);
            map.evaluteMap("src/sem_mapping/semmapping/maps/final_map_stats/world2_with_noise.csv");
        }
        else if (command == "save") {
            std::string fname = readNext(in, it);
            std::cout << "Saving map file: " << fname << std::endl;
            std::ofstream file(fname);
            if (!file) {
                std::cout << "File could not be opened" << std::endl;
                continue;
            }
            if (map.writeMapData(file)) {
                std::cout << "Map saving successfully" << std::endl;
            } else {
                std::cout << "Failed saving map" << std::endl;
            }
            file.close();
        } 
        else if (command == "exit") {
            std::cout << "Shutting down" << std::endl;
            break;
        } 
        else if (command == "clear") {
            // TODO
        } 
        else {
            std::cout << "Command \"" << command << "\" not recognized" << std::endl;
        }
    }
}
