#include <ros/ros.h>
#include <velodyne_msgs/VelodynePacket.h>
#include <velodyne_driver/input.h>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/rawdata.h>
#include <rosbag/bag.h>
#include <pcl_ros/transforms.h>
#include <tf/tfMessage.h>
#include <string>
#include <iostream>
#include <fstream>
#include <istream>
#include <sstream>
#include <map>
#include <cstdint>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
using namespace std;

// trim from left
inline std::string &ltrim(std::string &s, const char* t = " \t\n\r\f\v")
{
    s.erase(0, s.find_first_not_of(t));
    return s;
}

// trim from right
inline std::string &rtrim(std::string &s, const char* t = " \t\n\r\f\v")
{
    s.erase(s.find_last_not_of(t) + 1);
    return s;
}

// trim from left & right
inline std::string &trim(std::string &s, const char* t = " \t\n\r\f\v")
{
    return ltrim(rtrim(s, t), t);
}

const double deg2rad=3.14159265/180.0;


void ReadImuFile2bag(std::string file, rosbag::Bag &bag_out, ros::Time StartTime,ros::Time EndTime )
{
  std::cout<<"Parsing Imu file from  "<<StartTime<<" To " <<EndTime<< std::endl;
  sensor_msgs::Imu ImuMsg;
  ImuMsg.header.frame_id="imu_link";

  std::ifstream ifs(file.c_str());

  if(!ifs.is_open())
  {
    std::cout<<"could not open "<<file<<std::endl;
    return ;
  }

  bool calibrationMsgFound=false;
  double IMUCalibrationParams[3];

  int counterIMU=0;
  int counterGPS=0;

  bool BaseHourFound=false;
  uint64_t BaseHourMicrosec=0;
  uint64_t BaseHourMicrosecTopofHour=0;
  while(ifs)
  {
    std::string line;
    if(!std::getline(ifs,line))
      break;

    std::istringstream ss(line);
    std::vector <std::string> record;

    while (ss)
      {
         std::string s;
           ss >> s;

            if (!trim(s).empty())
                   record.push_back(s);
      }

    ros::Time MsgTime;
    if(record[0].compare("1462")==0)
    {
      // no estimate of orientation yet
     // ImuMsg.orientation_covariance={-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0};

      //uknown covariance
      ImuMsg.orientation_covariance=ImuMsg.angular_velocity_covariance=ImuMsg.linear_acceleration_covariance=
          {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

      uint64_t time_microsec_beg =  strtoull(record[5].c_str(),(char**)NULL,10);

      if(!BaseHourFound)
      {
        BaseHourMicrosec=time_microsec_beg;
        BaseHourMicrosecTopofHour=time_microsec_beg%3600000000;
        BaseHourFound=true;
      }

      uint64_t time_microsec_pasthour=(time_microsec_beg -BaseHourMicrosec)+ BaseHourMicrosecTopofHour;
      uint64_t sec=time_microsec_pasthour/1000000;
      uint64_t nanosec=(time_microsec_pasthour%1000000)*1000;

      ImuMsg.linear_acceleration.x=atof(record[6].c_str());
      ImuMsg.linear_acceleration.y=atof(record[7].c_str());
      ImuMsg.linear_acceleration.z=atof(record[8].c_str());

      ImuMsg.angular_velocity.x=atof(record[9].c_str());
      ImuMsg.angular_velocity.y=atof(record[10].c_str());
      ImuMsg.angular_velocity.z=atof(record[11].c_str());

      ImuMsg.header.stamp=ros::Time(sec,nanosec);
      MsgTime=ImuMsg.header.stamp;
      if(ImuMsg.header.stamp>=StartTime)
      {
        counterIMU++;
        bag_out.write("imu",ImuMsg.header.stamp,ImuMsg);
        std::cout<<"\rsaving Imu msg "<<ImuMsg.header.stamp;

      }
      else
      {
        std::cout<<"\rdiscard Imu msg "<<ImuMsg.header.stamp;
      }

    }
    else if(record[0].compare("508")==0)
    {
      uint64_t time_microsec_beg =  strtoull(record[4].c_str(),(char**)NULL,10);

      if(!BaseHourFound)
      {
        BaseHourMicrosec=time_microsec_beg;
        BaseHourMicrosecTopofHour=time_microsec_beg%3600000000;
        BaseHourFound=true;
      }
      uint64_t time_microsec_pasthour=(time_microsec_beg -BaseHourMicrosec)+ BaseHourMicrosecTopofHour;
      uint64_t sec=time_microsec_pasthour/1000000;
      uint64_t nanosec=(time_microsec_pasthour%1000000)*1000;

      // unknown covariance
      ImuMsg.orientation_covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

      //no velocity msg available
      ImuMsg.angular_velocity_covariance=ImuMsg.linear_acceleration_covariance=
          {-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0};


      double x= atof(record[5].c_str());
      double y= atof(record[6].c_str());
      double z= atof(record[7].c_str());

      double roll= deg2rad*atof(record[8].c_str());
      double pitch=deg2rad*atof(record[9].c_str());
      double yaw=deg2rad*atof(record[10].c_str());

      ImuMsg.orientation= tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);

      ImuMsg.header.stamp=ros::Time(sec,nanosec);
      MsgTime=ImuMsg.header.stamp;

      nav_msgs::Odometry odom;
      odom.header.frame_id="odom";
      odom.child_frame_id="base_link";
      odom.header.stamp=MsgTime;
      odom.pose.pose.position.x=x;
      odom.pose.pose.position.y=y;
      odom.pose.pose.position.z=z;
      odom.pose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);

      for(int i=0;i<36;i++)
      {
        odom.pose.covariance[i]=0.0; // uknown covariance
        odom.twist.covariance[i]=-1.0; //unknown data
      }

      if(MsgTime>=StartTime)
      {
        counterGPS++;
        bag_out.write("odom",odom.header.stamp,odom);
        std::cout<<"\rsaving GPS msg "<<odom.header.stamp;
      }else
      {
        std::cout<<"\rdiscard GPS msg "<<odom.header.stamp;
      }

    }
     else if(record[0].compare("642")==0)
    {
      for(int i=0;i<3;i++)
        IMUCalibrationParams[i]=atof(record[i+1].c_str());
    }

    if(MsgTime> EndTime)
    {
      std::cout<<"\n reached end of point cloud time"<<endl;
      break;
    }


  }

  ifs.close();
  std::cout<<"\nSaved Imu Messages in the bag "<<counterIMU<< " GPS Message "<<counterGPS<< endl;

}
inline std::string GetFileFolderName(const std::string &FilePath)
{
    size_t found;
    found = FilePath.find_last_of("/\\");

    return FilePath.substr(0,found);
}
inline std::string GetFileName(const std::string &FilePath)
{
    size_t found;
    found = FilePath.find_last_of("/\\");
    return FilePath.substr(found + 1);
}
inline std::string GetFileNameWithoutExtention(const std::string &FilePath)
{
    std::string filename = GetFileName(FilePath);
    size_t found;
    found = filename.find_last_of(".");
    return filename.substr(0,found);
}
/******************************************************************************
 */
//! Configuration parameters
struct Params
{
  //! Target frame ID
  //! - An empty value means to use the same frame ID as the input point cloud has...
  std::string frame_id;

  //! Range to accumulate particular Velodyne scans
  //! - Specifying two exactly similar values means to accumulate all the Velodyne points...
  double min_z, max_z;

  //! Angular resolution [degrees]
  double angular_res;

  //! Minimal range used to remove points close to the robot.
  //! - Negative value means that the minimum will be obtained from the data.
  double min_range;

  //! Default constructor
  Params()
    : frame_id("scan")
    , min_z(getDefaultMinZ())
    , max_z(getDefaultMaxZ())
    , angular_res(getDefaultAngularRes())
    , min_range(getDefaultMinRange())
  {}

  //! Default parameter values.
  static double getDefaultMinZ()
  {
    return -1000.0;
  }
  static double getDefaultMaxZ()
  {
    return 1000.0;
  }
  static double getDefaultAngularRes()
  {
    return 0.1;
  }
  static double getDefaultMinRange()
  {
    return -1.0;
  }
};

Params params_;
sensor_msgs::LaserScan::Ptr process(const velodyne_rawdata::VPointCloud &pcl_in_)
{
 ROS_INFO_STREAM_ONCE("LaserScan::process(): Point cloud received");

 // Output scan
 sensor_msgs::LaserScan::Ptr scan_out;
 scan_out = boost::make_shared<sensor_msgs::LaserScan>();

 // Calculate the number of output bins
 int num_of_bins = (params_.angular_res > 0.0f) ? int(360 / params_.angular_res) : 4096;
 float angular_res = 360.0f / num_of_bins;
 float inv_angular_res = 1.0f / angular_res;
 float rad_to_deg = 180.0f / float(3.141592);
 float range_min = 1e7f, range_max = -1e7f;

 // Initialize the simulated laser scan
 //scan_out->ranges.resize(num_of_bins);
 num_of_bins=36000;
 scan_out->ranges.resize(num_of_bins);
 scan_out->intensities.resize(num_of_bins);
 for (int i = 0; i < num_of_bins; ++i)
 {
   scan_out->ranges[i] = range_min;
   scan_out->intensities[i] = 0.0f;
 }

 // Create the simulated laser scan
 auto itEnd = pcl_in_.end();
 for (auto it = pcl_in_.begin(); it != itEnd; ++it)
 {

   if(it->laser_number!=15) // only the middle beam is considered
     continue;
//        ROS_INFO_STREAM("Point: " << it->x << ", " << it->y << ", " << it->z);

   // Check the point height
   //if (params_.min_z != params_.max_z)
  // {
   //  if (it->z < params_.min_z || it->z > params_.max_z)
    //   continue;
  // }

   // Conversion to the polar coordinates
   float mag = std::sqrt(it->x * it->x + it->y * it->y);
   float ang = std::atan2(it->y, it->x) * rad_to_deg;

   mag=it->distance;
   //ang= (float)it->azimuth/36000.0*2*3.141592 ;
//        float mag = cv::sqrt(it->x * it->x + it->y * it->y);
//        float ang = cv::fastAtan2(it->y, it->x); // precision ~0.3 degrees

//        ROS_INFO_STREAM("Polar coords: " << mag << ", " << ang);

   // Check the point distance
   //if (params_.min_range > 0.0 && mag < params_.min_range)
    // continue;

   // Find the corresponding bin
   /*int n = (ang + 180.0f) * inv_angular_res;
   if (n >= num_of_bins)
     n = num_of_bins - 1;
   else if (n < 0)
     n = 0;*/

//        ROS_INFO_STREAM("Bin num.: " << n);

   // Accumulate the value
   int n=it->azimuth;
   if (mag < scan_out->ranges[n])
   {

     scan_out->ranges[n] = mag;
     scan_out->intensities[n] = it->intensity;
   }

   // Overall stats
   range_min = (mag < range_min) ? mag : range_min;
   range_max = (mag > range_max) ? mag : range_max;
 }

 // Fill in all message members
 scan_out->angle_min =0;// -float(3.141592);
 scan_out->angle_max = 2.0*float(3.141592);
 scan_out->angle_increment = 0.01/180.0*3.141592;// angular_res / rad_to_deg;
 scan_out->range_min = range_min;
 scan_out->range_max = range_max;
 scan_out->scan_time = 0.1;      // TODO: get the value from Velodyne, fixed to 10Hz for now
 scan_out->time_increment = scan_out->scan_time / float(num_of_bins);

 // Publish the accumulated laser scan
 ROS_INFO_STREAM_ONCE("Publishing laser scan " << scan_out->header.stamp);

 return scan_out;
}


int main(int argc, char **argv)
{  
  if(argc<3)
  {
    std::cout <<"usage  pcapfile.pcap ImuFile.imu"<<std::endl;
    return 0;
  }
  std::string pcapfile=argv[1];
  std::string ImuFile=argv[2];
  std::string BaseName=GetFileFolderName(pcapfile)+"/" + GetFileNameWithoutExtention(pcapfile);
  std::string bagfile= argv[3];


  // Set up ROS.
  ros::init(argc, argv, "pcap2bag");

  if (!ros::ok())
    return 0;
  // Output bag
  rosbag::Bag bag_out(bagfile.c_str(),rosbag::bagmode::Write);



  //raeding pcap file
  ros::NodeHandle ns;
  ns.setParam("read_fast",true);
  ns.setParam("read_once",true);
  velodyne_driver::InputPCAP inputpcap(ns,2368,100,pcapfile);

  velodyne_rawdata::RawData data_;

  std::string calibfile="";//"/home/mehdi/velodyne_ws/src/velodyne/velodyne_pointcloud/params/32db.yaml";
  //std::string calibfile="/home/mehdi/velodyne_ws/src/velodyne/velodyne_pointcloud/params/VLP16db.yaml";

  // 754 Packets/Second for Last or Strongest mode 1508 for dual (VLP-16 User Manual)
  //const int PacketsPerRevolution=1808/10; //packet rate /HZ 32e

  //const int PacketsPerRevolution=1508/10; //packet rate /HZ vlp16
  bool dumpperrevolution=false;


  //unpack_simple function doesn't use these parameters
  //data_.setupOffline(calibfile,0,130);
  //data_.setParameters(0,130,0,0);
  velodyne_msgs::VelodynePacket vp;
  int counter=1;
  int RevolutionCounter=0;

  // allocate a point cloud with same time and frame ID as raw data
  velodyne_rawdata::VPointCloud inPc_;

  std::string frame_id="velodyne";
  ros::Time FirstPacketTime, EndPacketTime;
  std::ofstream sampledataofs;//(BaseName+"_sample.xyz")

  //2D laser scan
  sensor_msgs::LaserScan::Ptr scan_out=boost::make_shared<sensor_msgs::LaserScan>();
  int num_of_bins=36000;
  scan_out->ranges.resize(num_of_bins);
  scan_out->intensities.resize(num_of_bins);
  scan_out->angle_min =0.0;
  scan_out->angle_max =2.0*float(3.141592);
  scan_out->angle_increment = 0.01/180.0*3.141592;// angular_res / rad_to_deg;
  float range_min = 1e7f, range_max = -1e7f;
  for (int i = 0; i < num_of_bins; ++i)
  {
    scan_out->ranges[i] = range_min;
    scan_out->intensities[i] = 0.0f;
  }
  scan_out->range_min=range_min;
  scan_out->range_max=range_max;

  inPc_.points.clear();
  inPc_.width = 0;
  inPc_.height = 1;

  uint16_t previousrotation=1000;
  bool outputcsv=false;
  while (inputpcap.getPacket(&vp,0)!=-1)
  {

    // unpack the raw data
    const velodyne_rawdata::raw_packet_t* raw = (const velodyne_rawdata::raw_packet_t*) &vp.data[0];

    //check if new revolution is happening
    bool NewRevolution=false;
    if(raw->blocks[0].rotation>=previousrotation)
      NewRevolution=false;
    else
    {
      NewRevolution=true;
      RevolutionCounter++;
      sampledataofs.close();
      if(outputcsv)
        sampledataofs.open(BaseName+"_"+std::to_string(RevolutionCounter)+".csv");

    }
    previousrotation=raw->blocks[0].rotation;

    vp.stamp=ros::Time(vp.stamp.sec,vp.stamp.nsec);

    data_.unpack_simple(vp,inPc_, scan_out);

    if( (dumpperrevolution && NewRevolution) || !dumpperrevolution)
    {
      if(inPc_.width<1)
         continue;

        sensor_msgs::PointCloud2 pc2;
        pcl::toROSMsg(inPc_,pc2);

        //sensor_msgs::LaserScan::Ptr laserscan= process(inPc_);
        scan_out->header.stamp=pc2.header.stamp=vp.stamp;
        scan_out->header.frame_id=pc2.header.frame_id=frame_id;
        scan_out->scan_time = (vp.stamp-EndPacketTime).toSec();      // TODO: get the value from Velodyne, fixed to 10Hz for now
        scan_out->time_increment = scan_out->scan_time / float(num_of_bins);


        bag_out.write("points2",vp.stamp,pc2);

        if(dumpperrevolution)
          bag_out.write("scan",vp.stamp,*scan_out);

        EndPacketTime=vp.stamp;
        std::cout<<"\rend packet time "<<EndPacketTime ;

        if(outputcsv)
        {
          for (int k=0;k<inPc_.points.size();k++)
          {
            sampledataofs<<inPc_.points[k].x<<","<<inPc_.points[k].y<<","<<inPc_.points[k].z<<std::endl;
          }
        }

        //clearing data structures
        inPc_.points.clear();
        inPc_.width = 0;
        inPc_.height = 1;

        for (int i = 0; i < num_of_bins; ++i)
        {
          scan_out->ranges[i] = range_min;
          scan_out->intensities[i] = 0.0f;
        }
        scan_out->range_min=range_min;
        scan_out->range_max=range_max;


    }

    if(counter==1)
    {
      FirstPacketTime=vp.stamp;
      std::cout<<"\nVelodyne start packet time "<<FirstPacketTime<<std::endl;
    }

    counter++;

    if(!ros::ok())
      break;

  }

  sampledataofs.close();
  //reading imu file
  ReadImuFile2bag(ImuFile,bag_out, FirstPacketTime, EndPacketTime);
  bag_out.close();

}
