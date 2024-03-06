
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>



#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/normal_3d.h>  // Include normal estimation header
#include <pcl/features/fpfh.h>


#include <boost/thread/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

#include <iostream>

#include <algorithm>

namespace fs = boost::filesystem;

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
    R0.1 ->
    R0.5 ->
    R1 -> 0, 2, 16, 17, 18, 46, 83, 85, 86 
*/
void extractDescriptors(
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
    harris.setRadius(0.5); // default 0.1, determina el tamaño de los descriptores
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


void pc2map(std::string path){

    // 1: Transformación total T T = I
    // 2: Mapa 3D M
    // 3: para i=1 hasta t-1 hacer

    std::vector<fs::path> files;
    for (const auto& entry : fs::directory_iterator(path)) {
        files.push_back(entry.path());
    }
    // sorts the files alphabetically
    std::sort(files.begin(), files.end());

    int i=0;
    for (const auto& entry : files) {
        std::cout << entry << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);


        if (pcl::io::loadPCDFile<pcl::PointXYZ> (entry.string(), *cloud) == -1){
            PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
            return;
        }

        std::cout << "Loaded "
                    << cloud->width * cloud->height
                    << " data points from test_pcd.pcd with the following fields: "
                    << std::endl;


        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors(new pcl::PointCloud<pcl::FPFHSignature33>);
        extractDescriptors(cloud, keypoints, descriptors);

        std::cout << "Number of keypoints: " << keypoints->size() << std::endl;
        std::cout << "Number of descriptors: " << descriptors->size() << std::endl;

        i++;
    }
        // 4:
        // Extraer características C i = {c i 1 , c i 2 , ..., c im } de los datos x i .
        // i+1
        // i+1
        // 5:
        // Extraer características C i+1 = {c i+1
        // 1 , c 2 , ..., c k } de los datos x i+1 .
        // 6:
        // Encontrar emparejamientos P = {p 1 , p 2 , ..., p l } entre las características
        // C i y C i+1 . Cada emparejamiento es un par de características de cada
        // conjunto de datos p j = {c ia , c i+1
        // b }
        // 7:
        // Obtener la mejor transformación T i que explican los emparejamientos.
        // 8:
        // Obtener la transformación total T T = T T ∗ T i .
        // 9:
        // Aplicar al conjunto de datos C i+1 la transformación T T para colocar-
        // los en el sistema de coordenadas de C 1 . Acumular el conjunto de datos
        // transformados en M .
    // 10: fin para
    // 11: devolver M

}



int main(){
    pc2map("/home/alu/Escritorio/VAR/P1/point_data/");
}