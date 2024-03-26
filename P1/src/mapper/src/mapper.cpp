#include <iostream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>

#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/normal_3d.h> 
#include <pcl/features/fpfh.h>
#include <pcl/features/3dsc.h>

#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/ia_ransac.h>

#include <boost/thread/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

#include <algorithm>
#include <Eigen/Dense>



namespace fs = boost::filesystem;

const int ICP_MAX_ITER=50;

const float LEAF_SIZE = 0.1;

const float NORMAL_RADIUS = 0.2;

// KEYPOINTS //
// HARRIS
const float HARRIS_RADIUS = 0.1;
const float HARRIS_THR = 0.5;
// ISS
const float ISS_THR_21 = 0.95;
const float ISS_THR_32 = 0.95;
const float ISS_SAL_RADIUS = 6 * 0.05;
const float ISS_NON_MAX_RADIUS = 4 * 0.05;
const int ISS_MIN_NEIGHBORS = 5;
const int ISS_NUM_OF_THREADS = 4;

// DESCRIPTORS //
// FPFH
const float FPFH_RADIUS = 0.2;
// 3DSC
const float THREEDSC_RADIUS = 0.1;

// RANSAC
const int RANSAC_MAX_ITER = 5000;
const float RANSAC_MAX_DISTANCE = 0.05;
const float RANSAC_TRANSFORMATION_EPSILON = 1e-8;
const float RANSAC_FITNES_EPSILON = 1.0;
const float RANSAC_OUTLIER_THR = 0.1;


void extractKeypoints_harris_and_fpfh(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints,
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors
    );
void extractKeypoints_iss_and_fpfh(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints,
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors
    );
void extractKeypoints_harris_and_3dsc(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints,
        pcl::PointCloud<pcl::ShapeContext1980>::Ptr descriptors
    );
void extractKeypoints_iss_and_3dsc(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints,
        pcl::PointCloud<pcl::ShapeContext1980>::Ptr descriptors
    );

typedef pcl::FPFHSignature33 DescriptorType;
//typedef pcl::ShapeContext1980 DescriptorType;
typedef void (*FunctionPtr)(
        pcl::PointCloud<pcl::PointXYZ>::Ptr,
        pcl::PointCloud<pcl::PointXYZI>::Ptr,
        pcl::PointCloud<DescriptorType>::Ptr
    );
const FunctionPtr extraction_function = &extractKeypoints_iss_and_fpfh;



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



pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));

void clearVisualization() {
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    viewer->removeAllCoordinateSystems();
}

void extractKeypoints_harris_and_fpfh(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints,
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors
    ){
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);


    // Compute normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(NORMAL_RADIUS); 
    ne.compute(*normals);

    // Harris Keypoint Detection
    pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> harris;
    harris.setInputCloud(cloud);
    harris.setNormals(normals);
    harris.setNonMaxSupression(true); // poner a TRUE si se encuentran demasiados
    harris.setRadius(HARRIS_RADIUS); // determina el tamaño de los descriptores
    harris.setThreshold(HARRIS_THR); // menor thr == mas kps
    harris.compute(*keypoints);

    // FPFH Descriptor computation
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*keypoints, *keypointsXYZ); // FPFH requiere de una nube de puntos XYZ, no XYZI
    fpfh.setInputCloud(keypointsXYZ);
    fpfh.setInputNormals(normals);
    fpfh.setSearchSurface(cloud);
    fpfh.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    fpfh.setRadiusSearch(FPFH_RADIUS);
    fpfh.compute(*descriptors);
}



void extractKeypoints_iss_and_fpfh(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints,
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors
) {
    // Compute normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(0.1);
    ne.compute(*normals);

    // ISS Keypoint Detection
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> iss;
    iss.setInputCloud(cloud);
    iss.setSearchMethod(tree);
    iss.setSalientRadius(ISS_SAL_RADIUS); 
    iss.setNonMaxRadius(ISS_NON_MAX_RADIUS); 
    iss.setThreshold21(ISS_THR_21); 
    iss.setThreshold32(ISS_THR_32); 
    iss.setMinNeighbors(ISS_MIN_NEIGHBORS); 
    iss.setNumberOfThreads(ISS_NUM_OF_THREADS); 
    iss.compute(*keypoints);

    // FPFH Descriptor computation
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*keypoints, *keypointsXYZ);
    fpfh.setInputCloud(keypointsXYZ);
    fpfh.setInputNormals(normals);
    fpfh.setSearchSurface(cloud);
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(FPFH_RADIUS); 
    fpfh.compute(*descriptors);
}



void extractKeypoints_iss_and_3dsc(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints,
        pcl::PointCloud<pcl::ShapeContext1980>::Ptr descriptors
    ) {
    // Compute normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(NORMAL_RADIUS);
    ne.compute(*normals);

    // ISS Keypoint Detection
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> iss;
    iss.setInputCloud(cloud);
    iss.setSearchMethod(tree);
    iss.setSalientRadius(ISS_SAL_RADIUS); 
    iss.setNonMaxRadius(ISS_NON_MAX_RADIUS); 
    iss.setThreshold21(ISS_THR_21); 
    iss.setThreshold32(ISS_THR_32); 
    iss.setMinNeighbors(ISS_MIN_NEIGHBORS); 
    iss.setNumberOfThreads(ISS_NUM_OF_THREADS); 
    iss.compute(*keypoints);


    // 3DSC Descriptor computation
    pcl::search::KdTree<pcl::PointXYZI>::Ptr desc_tree(new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::ShapeContext3DEstimation<pcl::PointXYZI, pcl::Normal, pcl::ShapeContext1980> sc3d;
    sc3d.setInputCloud(keypoints);
    sc3d.setInputNormals(normals);
    sc3d.setSearchMethod(desc_tree);
    sc3d.setRadiusSearch(THREEDSC_RADIUS); 
    sc3d.compute(*descriptors);
}


void extractKeypoints_harris_and_3dsc(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints,
        pcl::PointCloud<pcl::ShapeContext1980>::Ptr descriptors
    ) {
    // Compute normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(NORMAL_RADIUS);
    ne.compute(*normals);

    // Harris Keypoint Detection
    pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> harris;
    harris.setInputCloud(cloud);
    harris.setNormals(normals);
    harris.setNonMaxSupression(true); // poner a TRUE si se encuentran demasiados
    harris.setRadius(HARRIS_RADIUS); // determina el tamaño de los descriptores
    harris.setThreshold(HARRIS_THR); // menor thr == mas kps
    harris.compute(*keypoints);

    // Compute normals for keypoints
    pcl::PointCloud<pcl::Normal>::Ptr kps_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_kps;
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*keypoints, *keypointsXYZ);
    ne_kps.setInputCloud(keypointsXYZ);
    ne_kps.setRadiusSearch(0.1); 
    ne_kps.compute(*kps_normals);

    pcl::PointCloud<pcl::PointXYZI>::Ptr search_surface(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*cloud, *search_surface);

    // 3DSC Descriptor computation
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::ShapeContext3DEstimation<pcl::PointXYZI, pcl::Normal, pcl::ShapeContext1980> sc3d;
    sc3d.setInputCloud(search_surface);
    //sc3d.setInputKeypoints(keypoints);
    sc3d.setInputNormals(normals);
    sc3d.setSearchSurface(search_surface); 
    sc3d.setSearchMethod(tree);
    sc3d.setRadiusSearch(THREEDSC_RADIUS); 
    sc3d.compute(*descriptors);
}

//=================================================================================================================================
//=================================================================================================================================
//=================================================================================================================================

/*Eigen::Matrix4f pairKeypoints_BAD(
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_source,
    pcl::PointCloud<DescriptorType>::Ptr descriptors_source,
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_target,
    pcl::PointCloud<DescriptorType>::Ptr descriptors_target
) {
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI>::Ptr ransac(new pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI>);
    ransac->setInputSource(keypoints_source);
    ransac->setInputTarget(keypoints_target);
    ransac->setInlierThreshold(0.05); // Adjust this threshold based on your application
    ransac->setMaximumIterations(1000); // Adjust maximum iterations as needed

    // Set correspondences based on descriptors
    pcl::registration::CorrespondenceEstimation<DescriptorType, DescriptorType>::Ptr correspondence_estimation(new pcl::registration::CorrespondenceEstimation<DescriptorType, DescriptorType>);
    correspondence_estimation->setInputSource(descriptors_source);
    correspondence_estimation->setInputTarget(descriptors_target);
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
    correspondence_estimation->determineCorrespondences(*correspondences);

    // Apply RANSAC to estimate the transformation
    ransac->setInputCorrespondences(correspondences);
    ransac->setInputTransformation(Eigen::Matrix4f::Identity()); // Initial transformation

    // Perform registration
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZI, pcl::PointXYZI> transformation_estimation;
    Eigen::Matrix4f transformation_matrix;
    transformation_estimation.estimateRigidTransformation(*keypoints_source, *keypoints_target, *ransac->getInliers(), transformation_matrix);

    return transformation_matrix;

}*/


Eigen::Matrix4f pairKeypoints(
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_source,
        pcl::PointCloud<DescriptorType>::Ptr descriptors_source,
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_target,
        pcl::PointCloud<DescriptorType>::Ptr descriptors_target
    ){
    // Initialize SampleConsensusInitialAlignment
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZI, pcl::PointXYZI, pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputSource(keypoints_source);
    sac_ia.setInputTarget(keypoints_target);
    sac_ia.setSourceFeatures(descriptors_source);
    sac_ia.setTargetFeatures(descriptors_target);

    // Set parameters for SAC-IA
    sac_ia.setMinSampleDistance(0.05); // Minimum distance between samples
    sac_ia.setMaxCorrespondenceDistance(0.1); // Maximum correspondence distance (where correspondences are ignored)
    sac_ia.setMaximumIterations(500); // Maximum number of iterations
    sac_ia.setEuclideanFitnessEpsilon(1e-5); // Fitness epsilon (convergence criterion)
    sac_ia.align(*keypoints_source);

    // Obtain the initial transformation from SAC-IA
    Eigen::Matrix4f initial_transformation = sac_ia.getFinalTransformation();

    // Apply SampleConsensusPrerejective using the initial transformation
    pcl::SampleConsensusPrerejective<pcl::PointXYZI, pcl::PointXYZI, pcl::FPFHSignature33> sac_pr;
    sac_pr.setInputSource(keypoints_source);
    sac_pr.setInputTarget(keypoints_target);
    sac_pr.setSourceFeatures(descriptors_source);
    sac_pr.setTargetFeatures(descriptors_target);
    sac_pr.setMaximumIterations(10000); // Maximum number of iterations
    sac_pr.setCorrespondenceRandomness(20); // Number of randomly selected correspondences to use
    sac_pr.setSimilarityThreshold(0.1); // Threshold for nearest neighbor clustering
    sac_pr.setMaxCorrespondenceDistance(0.1); // Maximum correspondence distance
    sac_pr.setInlierFraction(0.25); // Required inlier fraction for a successful alignment
    sac_pr.align(*keypoints_source);
    return sac_pr.getFinalTransformation();
    
    //Eigen::Matrix4f refined_transformation = sac_pr.getFinalTransformation();
    //Eigen::Matrix4f final_transformation = refined_transformation * initial_transformation;
    //return final_transformation;
}
// source = actual, target = previous
Eigen::Matrix4f pairKeypoints_HUH(
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_source,
        pcl::PointCloud<DescriptorType>::Ptr descriptors_source,
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_target,
        pcl::PointCloud<DescriptorType>::Ptr descriptors_target
    ){
    pcl::SampleConsensusPrerejective<pcl::PointXYZI, pcl::PointXYZI, DescriptorType> align;

    align.setInputSource(keypoints_source);
    align.setSourceFeatures(descriptors_source);
    align.setInputTarget(keypoints_target);
    align.setTargetFeatures(descriptors_target);

    align.setMaximumIterations(RANSAC_MAX_ITER);// 1000
    align.setMaxCorrespondenceDistance(RANSAC_MAX_DISTANCE); // 0.1
    align.setTransformationEpsilon(RANSAC_TRANSFORMATION_EPSILON); // 1e-8
    align.setEuclideanFitnessEpsilon(RANSAC_FITNES_EPSILON); // 1
    align.setRANSACOutlierRejectionThreshold(RANSAC_OUTLIER_THR); // 0.1

    pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>);
    align.align(*aligned);

    if (align.hasConverged()) {
        Eigen::Matrix4f transformation = align.getFinalTransformation();
        return transformation;
    } else {
        // Alignment failed
        std::cout << "FAILED alignment" << std::endl;
        return Eigen::Matrix4f::Identity();
    }
}

void pc2map(std::string path){

    pcl::PointCloud<pcl::PointXYZI>::Ptr previous_kps(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<DescriptorType>::Ptr previous_dscs(new pcl::PointCloud<DescriptorType>);
    
    Eigen::Matrix4f total_transformation = Eigen::Matrix4f::Identity();
    std::cout << total_transformation << std::endl;
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
        for (const auto& point : *cloud) {
            if (!pcl::isFinite(point)) {
                std::cout << "Found invalid point in cloud: (" << point.x << ", " << point.y << ", " << point.z << "), intensity: " << std::endl;
            }
        }

        // pcl::PassThrough<pcl::PointXYZ> pass;
        // pass.setInputCloud(cloud);
        // pass.setFilterFieldName("y");
        // pass.setFilterLimits(-10, 0); // filters the floor by deleting any y value above 0.114
        // pass.filter(*cloud);


        // keypoints are extracted
        pcl::PointCloud<pcl::PointXYZI>::Ptr kps(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<DescriptorType>::Ptr dscs(new pcl::PointCloud<DescriptorType>);
        
        extraction_function(cloud, kps, dscs);

        std::cout << "Number of keypoints: " << kps->size() << std::endl;
        std::cout << "Number of descriptors: " << dscs->size() << std::endl;
        for (const auto& point : *kps) {
            if (!pcl::isFinite(point)) {
                std::cout << "Found invalid point in keypoints: (" << point.x << ", " << point.y << ", " << point.z << "), intensity: " << std::endl;
            }
        }
        if(i>0){
            std::cout << "Pairing... " << std::endl;

            // the i matrix is calculated on pairKeypoints and then multiplied by the total
            //total_transformation = pairKeypoints(kps, dscs, previous_kps, previous_dscs) * total_transformation; // original: Ti*Tt - mal
            //total_transformation = pairKeypoints(previous_kps, previous_dscs, kps, dscs) * total_transformation; // swap source y target
            total_transformation = total_transformation * pairKeypoints(kps, dscs, previous_kps, previous_dscs); // Tt * Ti 
            //total_transformation = total_transformation * pairKeypoints(previous_kps, previous_dscs, kps, dscs); // swap source y target

            // the total is applied to the whole cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*cloud, *cloud_transformed, total_transformation);

            // the transformed cloud is added to the map
            *map = *map + *cloud_transformed; 

            //std::cout << total_transformation << std::endl;
            std::cout << "-- MAP now contains " << map->size() << " points.";
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setInputCloud(map);
            vg.setLeafSize(LEAF_SIZE,LEAF_SIZE,LEAF_SIZE);
            vg.filter(*map); 
            std::cout << " Reduced to " <<  map->size() << " points." << std::endl;

            clearVisualization();
            viewer->addPointCloud(map, "map");
        }else{
            *map = *map + *cloud; 
            std::cout << "1st iter, pairing can't be done." << std::endl;
        }
        // kps are copied for next iter
        *previous_kps = *kps;
        *previous_dscs = *dscs;

        i++;
        viewer->spinOnce();
    }
        
    // 10: fin para
    // 11: devolver M

}



int main(){
    //pc2map("/home/alu/Escritorio/VAR/P1/point_data/");
    pc2map("/home/alu/Escritorio/VAR/P1/point_data_old_2/");
    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
}