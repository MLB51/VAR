#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/registration/sample_consensus_prerejective.h>

#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/normal_3d.h> 
#include <pcl/features/fpfh.h>

#include <pcl/registration/registration.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/ia_ransac.h> // Include the RANSAC-based registration class

#include <boost/thread/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

#include <iostream>

#include <algorithm>

#include <Eigen/Dense>

namespace fs = boost::filesystem;

const int ICP_MAX_ITER=50;



/*
Feature extraction: 
    Harris 3D: pcl::HarrisKeypoint3D
    ISS: pcl::ISSKeypoint3D
    SIFT: pcl::SIFTKeypoint
    Normal: pcl::NormalEstimation

Descriptor Extraction:
    FPH: pcl::FPFHEstimation
    SHOT: pcl::SHOTEstimation
    VFH: pcl::VFHEstimation
    3DSC: pcl::ShapeContext3DEstimation

*/


/*
Nubes problematicas (?)
    R0.1 -> 0,2, 4, 5, 14, 15, 16, 17, 18, 21, 23, 24 27, 29, 35 ,41,42,44,45,46, 49, 52,53, 55, 60,79,80,81,83,84,85,86,87, 88, 90
    R0.5 -> 
    R1 -> 0, 2, 16, 17, 18, 46, 83, 85, 86
*/
void extractKeypoints(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints,
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors
    ){
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);


    // Compute normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(0.03);
    ne.compute(*normals);

    // Harris Keypoint Detection
    pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> harris;
    harris.setInputCloud(cloud);
    harris.setNormals(normals);
    harris.setNonMaxSupression(true); // pasar a TRUE si se encuentran demasiados
    harris.setRadius(0.1); // default 0.1, determina el tamaño de los descriptores
    harris.setThreshold(0.01); // default 0.01, menor thr == mas kps
    harris.compute(*keypoints);

    // FPFH Descriptor computation
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*keypoints, *keypointsXYZ);
    fpfh.setInputCloud(keypointsXYZ);
    fpfh.setInputNormals(normals);
    fpfh.setSearchSurface(cloud);
    fpfh.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    fpfh.setRadiusSearch(0.05);
    fpfh.compute(*descriptors);
}

// source = actual, target = previous
Eigen::Matrix4f pairKeypoints(
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_source,
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors_source,
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_target,
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors_target
    ){
    pcl::SampleConsensusPrerejective<pcl::PointXYZI, pcl::PointXYZI, pcl::FPFHSignature33> align;

    align.setInputSource(keypoints_source);
    align.setSourceFeatures(descriptors_source);
    align.setInputTarget(keypoints_target);
    align.setTargetFeatures(descriptors_target);

    pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>);
    align.align(*aligned);

    if (align.hasConverged()) {
        Eigen::Matrix4f transformation = align.getFinalTransformation();
        return transformation;
    } else {
        // Alignment failed
        return Eigen::Matrix4f::Identity();
    }
}

void pc2map(std::string path){

    pcl::PointCloud<pcl::PointXYZI>::Ptr previous_kps(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr previous_dscs(new pcl::PointCloud<pcl::FPFHSignature33>);
    
    Eigen::Matrix4f total_transformation = Eigen::Matrix4f::Identity();
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>);
    
    // gets all the files paths
    std::vector<fs::path> files;
    for (const auto& entry : fs::directory_iterator(path)) {
        files.push_back(entry.path());
    }
    // sorts the files alphabetically as they are saved like cloud_<i>
    std::sort(files.begin(), files.end());

    int i=0;
    for (const auto& entry : files) {
        std::cout << std::endl << entry << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);


        if (pcl::io::loadPCDFile<pcl::PointXYZ> (entry.string(), *cloud) == -1){
            std::cerr << "Couldn't read file " << entry.string() << std::endl;
            return;
        }

        std::cout << "Loaded cloud with " << cloud->width * cloud->height << " data points" <<std::endl;
        
        // keypoints are extracted
        pcl::PointCloud<pcl::PointXYZI>::Ptr kps(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr dscs(new pcl::PointCloud<pcl::FPFHSignature33>);
        extractKeypoints(cloud, kps, dscs);
        std::cout << "Number of keypoints: " << kps->size() << std::endl;
        std::cout << "Number of descriptors: " << dscs->size() << std::endl;

        if(i>0){
            std::cout << "Pairing... " << std::endl;

            // the i matrix is calculated on pairKeypoints and then multiplied by the total
            total_transformation = total_transformation * pairKeypoints(kps, dscs, previous_kps, previous_dscs);

            // the total is applied to the whole cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*cloud, *cloud_transformed, total_transformation);

            // the transformed cloud is added to the map
            *map = *map + *cloud_transformed; 
            
        }else{

            std::cout << "1st iter, pairing can't be done." << std::endl;
        }
        // por ultimo se copian los kps actuales para la siguiente iter
        *previous_kps = *kps;
        *previous_dscs = *dscs;

        i++;
    }
        
    // 10: fin para
    // 11: devolver M

}



int main(){
    pc2map("/home/alu/Escritorio/VAR/P1/point_data/");
}