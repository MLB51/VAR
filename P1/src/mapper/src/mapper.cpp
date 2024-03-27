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
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/shot.h>
#include <pcl/features/vfh.h>

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

const std::string POINT_CLOUDS_PATH = "/home/alu/Escritorio/VAR/P1/point_data/";
pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
const float LEAF_SIZE = 0.1;
const float NORMAL_RADIUS = 0.2;


// KEYPOINTS //
// HARRIS
const float HARRIS_RADIUS = 0.1;
const float HARRIS_THR = 0.3;

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
const float THREEDSC_RADIUS = 0.2;
//SHOT
const float SHOT_RADIUS = 0.2;
// VHF
const float VHF_RADIUS = 0.2;
// PFH
const float PFH_RADIUS = 0.2;

// RANSAC
const int RANSAC_MAX_ITER = 5000;
const int RANSAC_INIT_MAX_ITER = 1000;
const float RANSAC_MAX_DISTANCE = 0.1; // distancia a partir de la que se considera outlier durante alg
const float RANSAC_OUTLIER_THR = 0.15; // distancia a partir de la cual se considera outlier al final, cuando se calcula trnasformacion
const float RANSAC_TRANSFORMATION_EPSILON = 1e-8; // epsilon que marca convergencia en matriz
const float RANSAC_FITNES_EPSILON = 0.05; // epsilon que marca convergencia en distancias 
const float RANSAC_MIN_SAMPLE_DIST = 0.1; // distancia minima entre samples
const float RANSAC_SIMILARITY_THR = 0.9; // threshold de similitud entre descriptroes
const float RANSAC_INLIER_RATE = 0.25; // % de inliers para considerar que ha ido bien

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
void extractKeypoints_iss_and_shot(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints,
        pcl::PointCloud<pcl::SHOT352>::Ptr descriptors
    );

void extractKeypoints_iss_and_vhf(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints,
        pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors
    );

void extractKeypoints_iss_and_pfh(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints,
        pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors
    );

// change defintion to use other descriptors
typedef pcl::FPFHSignature33 DescriptorType;
//typedef pcl::ShapeContext1980 DescriptorType;
//typedef pcl::SHOT352 DescriptorType;
//typedef pcl::VFHSignature308 DescriptorType;
//typedef pcl::PFHSignature125 DescriptorType;

typedef void (*FunctionPtr)(
        pcl::PointCloud<pcl::PointXYZ>::Ptr,
        pcl::PointCloud<pcl::PointXYZI>::Ptr,
        pcl::PointCloud<DescriptorType>::Ptr
    );
const FunctionPtr extraction_function = &extractKeypoints_iss_and_fpfh;

//=================================================================================================================================
//=================================================================================================================================
//=================================================================================================================================

void extractKeypoints_harris_and_fpfh(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints,
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors
    ){
    // Normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(NORMAL_RADIUS); 
    ne.compute(*normals);

    // Harris
    pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> harris;
    harris.setInputCloud(cloud);
    harris.setNormals(normals);
    harris.setNonMaxSupression(true); // poner a TRUE si se encuentran demasiados
    harris.setRadius(HARRIS_RADIUS); // determina el tamaño de los descriptores
    harris.setThreshold(HARRIS_THR); // menor thr == mas kps
    harris.compute(*keypoints);

    // FPFH
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
    // Normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(NORMAL_RADIUS);
    ne.compute(*normals);

    // ISS
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> iss;
    iss.setInputCloud(cloud);
    iss.setSearchMethod(tree);
    iss.setSalientRadius(ISS_SAL_RADIUS); // radio del area de las caracteristicas a detectar
    iss.setNonMaxRadius(ISS_NON_MAX_RADIUS); // radio dentro del que se aplica supresion de no maximos
    iss.setThreshold21(ISS_THR_21);  // thr para relacion entre autovalores 2 y 1
    iss.setThreshold32(ISS_THR_32);  // thr para relacion entre autovalores 3 y 2
    iss.setMinNeighbors(ISS_MIN_NEIGHBORS); // minimo de vecinos en el area de busqueda
    iss.setNumberOfThreads(ISS_NUM_OF_THREADS); 
    iss.compute(*keypoints);

    // FPFH
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
    // Normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(NORMAL_RADIUS);
    ne.compute(*normals);

    // ISS
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

    // Normals for keypoints
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*keypoints, *keypointsXYZ);
    pcl::PointCloud<pcl::Normal>::Ptr kps_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_kps;
    ne_kps.setInputCloud(keypointsXYZ);
    ne_kps.setRadiusSearch(NORMAL_RADIUS); 
    ne_kps.compute(*kps_normals);

    // 3DSC 
    pcl::search::KdTree<pcl::PointXYZ>::Ptr desc_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::ShapeContext3DEstimation<pcl::PointXYZ, pcl::Normal, pcl::ShapeContext1980> sc3d;
    sc3d.setInputCloud(keypointsXYZ);
    sc3d.setInputNormals(kps_normals);
    sc3d.setSearchSurface(keypointsXYZ); 
    sc3d.setSearchMethod(desc_tree);
    sc3d.setRadiusSearch(THREEDSC_RADIUS); 
    sc3d.compute(*descriptors);
}

void extractKeypoints_harris_and_3dsc(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints,
        pcl::PointCloud<pcl::ShapeContext1980>::Ptr descriptors
    ) {
    // Normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(NORMAL_RADIUS);
    ne.compute(*normals);

    // Harris
    pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> harris;
    harris.setInputCloud(cloud);
    harris.setNormals(normals);
    harris.setNonMaxSupression(true); // poner a TRUE si se encuentran demasiados
    harris.setRadius(HARRIS_RADIUS); // determina el tamaño de los descriptores
    harris.setThreshold(HARRIS_THR); // menor thr == mas kps
    harris.compute(*keypoints);

    // Normals for keypoints
    pcl::PointCloud<pcl::Normal>::Ptr kps_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_kps;
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*keypoints, *keypointsXYZ);
    ne_kps.setInputCloud(keypointsXYZ);
    ne_kps.setRadiusSearch(NORMAL_RADIUS); 
    ne_kps.compute(*kps_normals);

    // 3DSC
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::ShapeContext3DEstimation<pcl::PointXYZ, pcl::Normal, pcl::ShapeContext1980> sc3d;
    sc3d.setInputCloud(keypointsXYZ);
    sc3d.setInputNormals(kps_normals);
    sc3d.setSearchMethod(tree);
    sc3d.setRadiusSearch(THREEDSC_RADIUS); 
    sc3d.compute(*descriptors);
}

void extractKeypoints_iss_and_shot(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints,
        pcl::PointCloud<pcl::SHOT352>::Ptr descriptors
    ) {

    // ISS
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

    // Normals for keypoints
    pcl::PointCloud<pcl::Normal>::Ptr kps_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_kps;
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*keypoints, *keypointsXYZ);
    ne_kps.setInputCloud(keypointsXYZ);
    ne_kps.setRadiusSearch(NORMAL_RADIUS); 
    ne_kps.compute(*kps_normals);

    // SHOT
    pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
    shot.setInputCloud(keypointsXYZ);
    shot.setInputNormals(kps_normals);
    shot.setRadiusSearch(SHOT_RADIUS);
    shot.compute(*descriptors);
}

void extractKeypoints_iss_and_vhf(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints,
        pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors
    ) {

    // ISS
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

    // Normals for keypoints
    pcl::PointCloud<pcl::Normal>::Ptr kps_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_kps;
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*keypoints, *keypointsXYZ);
    ne_kps.setInputCloud(keypointsXYZ);
    ne_kps.setRadiusSearch(NORMAL_RADIUS); 
    ne_kps.compute(*kps_normals);

    // VFH
    pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr dsc_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    vfh.setInputCloud(keypointsXYZ);
    vfh.setInputNormals(kps_normals);
    vfh.setSearchMethod(dsc_tree);
    vfh.setRadiusSearch(VHF_RADIUS);
    vfh.compute(*descriptors);

}

void extractKeypoints_iss_and_pfh(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints,
        pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors
    ) {
    // Normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(NORMAL_RADIUS);
    ne.compute(*normals);

    // ISS
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

    // PFH
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*keypoints, *keypointsXYZ);
    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
    pfh.setInputCloud(keypointsXYZ);
    pfh.setInputNormals(normals);
    pfh.setSearchSurface(cloud);
    pfh.setSearchMethod(tree);
    pfh.setRadiusSearch(PFH_RADIUS); // radio del area que se analiza
    pfh.compute(*descriptors);

}
//=================================================================================================================================
//=================================================================================================================================
//=================================================================================================================================

Eigen::Matrix4f pairKeypoints_with_prealignment(
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_source,
        pcl::PointCloud<DescriptorType>::Ptr descriptors_source,
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_target,
        pcl::PointCloud<DescriptorType>::Ptr descriptors_target
    ){
    // applies prealignment for RANSAC
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZI, pcl::PointXYZI, DescriptorType> sac_ia;
    sac_ia.setInputSource(keypoints_source);
    sac_ia.setInputTarget(keypoints_target);
    sac_ia.setSourceFeatures(descriptors_source);
    sac_ia.setTargetFeatures(descriptors_target);
    sac_ia.setMinSampleDistance(RANSAC_MIN_SAMPLE_DIST);
    sac_ia.setMaxCorrespondenceDistance(RANSAC_MAX_DISTANCE);
    sac_ia.setMaximumIterations(RANSAC_INIT_MAX_ITER);
    sac_ia.setEuclideanFitnessEpsilon(RANSAC_FITNES_EPSILON);
    sac_ia.align(*keypoints_source);
    Eigen::Matrix4f initial_transformation = sac_ia.getFinalTransformation();

    // applies RANSAC using the initial transformation
    pcl::SampleConsensusPrerejective<pcl::PointXYZI, pcl::PointXYZI, DescriptorType> sac_pr;
    sac_pr.setInputSource(keypoints_source);
    sac_pr.setInputTarget(keypoints_target);
    sac_pr.setSourceFeatures(descriptors_source);
    sac_pr.setTargetFeatures(descriptors_target);
    sac_pr.setMaximumIterations(RANSAC_MAX_ITER);
    sac_pr.setSimilarityThreshold(RANSAC_SIMILARITY_THR); // threshold de similitud entre descriptores
    sac_pr.setMaxCorrespondenceDistance(RANSAC_MAX_DISTANCE); // distancia a partir de la que se considera outlier durante alg
    sac_pr.setInlierFraction(RANSAC_INLIER_RATE); // % de inliers necesarios para considerar que ha ido bien
    sac_pr.setTransformationEpsilon(RANSAC_TRANSFORMATION_EPSILON); // epsilon de la transformacion a partir del cual converge
    sac_pr.setEuclideanFitnessEpsilon(RANSAC_FITNES_EPSILON); // epsilon de las distancias a partir del cual converge
    // distancia a partir de la cual se considera outlier al final, cuando se calcula transformacion
    sac_pr.setRANSACOutlierRejectionThreshold(RANSAC_OUTLIER_THR); 
    
    sac_pr.align(*keypoints_source);
    if (sac_pr.hasConverged()) {
        std::cout << "\033[1;32mSUCCES in alignment\033[0m" << std::endl;
    } else {
        // Alignment failed
        std::cout << "\033[1;31mFAILED alignment\033[0m" << std::endl;
    }
    Eigen::Matrix4f transformation = sac_pr.getFinalTransformation();
    return transformation * initial_transformation;
}

Eigen::Matrix4f pairKeypoints_without_prealignment(
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_source,
        pcl::PointCloud<DescriptorType>::Ptr descriptors_source,
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_target,
        pcl::PointCloud<DescriptorType>::Ptr descriptors_target
    ){
    pcl::SampleConsensusPrerejective<pcl::PointXYZI, pcl::PointXYZI, DescriptorType> sac_pr;
    sac_pr.setInputSource(keypoints_source);
    sac_pr.setInputTarget(keypoints_target);
    sac_pr.setSourceFeatures(descriptors_source);
    sac_pr.setTargetFeatures(descriptors_target);
    sac_pr.setMaximumIterations(RANSAC_MAX_ITER);
    sac_pr.setSimilarityThreshold(RANSAC_SIMILARITY_THR); // threshold de similitud entre descriptores
    sac_pr.setMaxCorrespondenceDistance(RANSAC_MAX_DISTANCE); // distancia a partir de la que se considera outlier durante alg
    sac_pr.setInlierFraction(RANSAC_INLIER_RATE); // % de inliers necesarios para considerar que ha ido bien
    sac_pr.setTransformationEpsilon(RANSAC_TRANSFORMATION_EPSILON); // epsilon de la transformacion a partir del cual converge
    sac_pr.setEuclideanFitnessEpsilon(RANSAC_FITNES_EPSILON); // epsilon de las distancias a partir del cual converge
    // distancia a partir de la cual se considera outlier al final, cuando se calcula transformacion
    sac_pr.setRANSACOutlierRejectionThreshold(RANSAC_OUTLIER_THR); 
    
    sac_pr.align(*keypoints_source);
    if (sac_pr.hasConverged()) {
        std::cout << "\033[1;32mSUCCES in alignment\033[0m" << std::endl;
    } else {
        // Alignment failed
        std::cout << "\033[1;31mFAILED alignment\033[0m" << std::endl;
    }
    Eigen::Matrix4f transformation = sac_pr.getFinalTransformation();
    return transformation;
}

//=================================================================================================================================
//=================================================================================================================================
//=================================================================================================================================

void clearVisualization() {
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    viewer->removeAllCoordinateSystems();
}
void pc2map(std::string path){
    // init Tt and Map
    Eigen::Matrix4f total_transformation = Eigen::Matrix4f::Identity();
    pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr previous_kps(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<DescriptorType>::Ptr previous_dscs(new pcl::PointCloud<DescriptorType>);

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

        // Filtering of the floor makes it worse so it's not used
        /*// pcl::PassThrough<pcl::PointXYZ> pass;
        // pass.setInputCloud(cloud);
        // pass.setFilterFieldName("y");
        // pass.setFilterLimits(-10, 0); // filters every point with y above 0, as this represents the floor
        // pass.filter(*cloud);*/
 
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
            Eigen::Matrix4f transformation_i = pairKeypoints_without_prealignment(kps, dscs, previous_kps, previous_dscs);
            total_transformation = total_transformation * transformation_i; // Tt * Ti 

            // the total is applied to the whole cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*cloud, *cloud_transformed, total_transformation);

            // the transformed cloud is added to the map
            *map = *map + *cloud_transformed; 

            std::cout << "-- MAP now contains " << map->size() << " points.";
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setLeafSize(LEAF_SIZE,LEAF_SIZE,LEAF_SIZE);
            vg.setInputCloud(map);
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
}

int main(){
    pc2map(POINT_CLOUDS_PATH);
    while (!viewer->wasStopped()) {
        viewer->spinOnce(); //loop until window is closed
    }
}