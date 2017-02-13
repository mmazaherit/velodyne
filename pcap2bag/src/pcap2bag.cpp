#include <ros/ros.h>
#include <velodyne_msgs/VelodynePacket.h>
#include <velodyne_driver/input.h>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/rawdata.h>
#include <rosbag/bag.h>
#include <pcl_ros/transforms.h>
#include <string>
#include <iostream>
#include <fstream>

using namespace std;
int main(int argc, char **argv)
{
  std::string pcapfile="/media/D_DRIVE/Data/SLAM/Maverick/4097.pcap";
  std::string bagfile="/home/mehdi/testbag.bag";


  // Set up ROS.
  ros::init(argc, argv, "pcap2bag");

  // Output bag
  rosbag::Bag bag_out(bagfile.c_str(),rosbag::bagmode::Write);

  ros::NodeHandle ns;

  velodyne_driver::InputPCAP inputpcap(ns,2368,100,pcapfile);




  velodyne_rawdata::RawData data_;

  std::string calibfile="/home/mehdi/velodyne_ws/src/velodyne/velodyne_pointcloud/params/32db.yaml";
  data_.setupOffline(calibfile,0,100);
  data_.setParameters(0,130,0,0);
  velodyne_msgs::VelodynePacket vp;
  int counter=0;

  // allocate a point cloud with same time and frame ID as raw data
  velodyne_rawdata::VPointCloud inPc_;


  while ( inputpcap.getPacket(&vp,0)!=-1)
  {

    // clear input point cloud to handle this packet
    inPc_.points.clear();
    inPc_.width = 0;
    inPc_.height = 1;
    std_msgs::Header header;
    header.stamp = vp.stamp;
    header.frame_id="map";
    pcl_conversions::toPCL(header, inPc_.header);

    // unpack the raw data
    data_.unpack(vp,inPc_);


    if(inPc_.width<1)
      continue;

    sensor_msgs::PointCloud2 pc2;
    //pcl::PCLPointCloud2 pcl2;

    //pcl::toPCLPointCloud2(inPc_ ,pcl2);
    //pcl_conversions::copyPCLPointCloud2MetaData(pcl2,pc2);
    pcl::toROSMsg(inPc_,pc2);

    bag_out.write("/map",ros::Time(vp.stamp),pc2);
    counter++;
    if(counter==1000)
      break;
    if(!ros::ok())
      break;
  }

  bag_out.close();
}
