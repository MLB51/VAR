#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <boost/foreach.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr visu_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
int cloud_counter=0;
void simpleVis ()
{
  	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	while(!viewer.wasStopped())
	{
		viewer.showCloud (visu_pc);
		boost::this_thread::sleep(boost::posix_time::milliseconds(500));
		
	}

}

void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>(*msg));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

	cout << "Puntos capturados: " << cloud->size() << endl;

	pcl::VoxelGrid<pcl::PointXYZRGB > vGrid;
	vGrid.setInputCloud (cloud);
	vGrid.setLeafSize (0.05f, 0.05f, 0.05f);
	vGrid.filter (*cloud_filtered);

	cout << "Puntos tras VG: " << cloud_filtered->size() << endl;
	
	visu_pc = cloud_filtered;
	if(visu_pc->size() > 0){
		std::string iter_string = std::to_string(cloud_counter++);
		if (iter_string.size() < 2) {iter_string = "0" + iter_string;}

		std::string filename = "/home/alu/Escritorio/VAR/P1/point_data/cloud_"+iter_string+".pcd";
		pcl::io::savePCDFileASCII(filename, *cloud_filtered);
	}
}
		

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sub_pcl");
	ros::NodeHandle nh;
	// aqui lo que hace es ir leyendo y printeando en pantalla
	ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth/points", 1, callback);

	// aqui se podria crear otro thread que en vez de mostrar construya el mapa
	boost::thread t(simpleVis);

	while(ros::ok())
	{
		ros::spinOnce();
	}

}
