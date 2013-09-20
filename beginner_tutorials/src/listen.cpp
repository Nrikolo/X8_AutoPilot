#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
 
class LaserScanFilter //Declare class structure
{
    LaserScanFilter();//Constructor
    ~LaserScanFilter();//Destructor
    ros::NodeHandle nh;
    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    sensor_msgs::LaserScan::ConstPtr scan_out;
    ros::Publisher  laser_scan_publisher;
    ros::Subscriber laser_scan_subscriber; 
    unsigned int mirror_right_start;
    unsigned int mirror_right_end;
    unsigned int mirror_left_start;
    unsigned int mirror_left_end;
    unsigned int mirror_width;
    float minimal_relevant_range;
    
};

//Constructor
LaserScanFilter::LaserScanFilter(ros::NodeHandle n)
{
    nh(n);
    mirror_right_start      = 16;
    mirror_right_end        = 27;
    mirror_left_start       = 1018;
    mirror_left_end         = 1029;
    mirror_width            = mirror_right_end-mirror_right_start;
    minimal_relevant_range  = 1; //meters
    laser_scan_publisher    = nh.advertise<sensor_msgs::LaserScan::ConstPtr>("scan_filtered",1);
    laser_scan_subscriber   = n.subscribe("scan", 100, &LaserScanFilter::scanCallback);
    
}

//Destructor
LaserScanFilter::~LaserScanFilter()
{
;//null
}

void LaserScanFilter::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
{
  scan_out = scan_msg;
  size_t num_ranges = scan_out->ranges.size();
  for(int itr=0 ; itr<=num_ranges ; itr++)
  {
      if((itr > mirror_right_start && itr <  mirror_right_end) || (itr>mirror_left_start && itr< mirror_left_end))
        {scan_out->ranges[itr]=scan_out->range_max;} // Filter the mirrors out by assigning maximal range to they samples
      if(scan_out->ranges[itr]=<minimal_relevant_range)
         {scan_out->ranges[itr]=scan_out->range_max;}// Filter out samples that are closer than 1 meter
  }
  laser_scan_publisher.publish(scan_out);//publish the filtered scan
  return;
}//end of callback

 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_filter_node");
  ros::NodeHandle n;
  LaserScanFilter Filter(n);
  ros::spin();
  return 0;
}


