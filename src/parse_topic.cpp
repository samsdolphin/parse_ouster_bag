#include <iostream>
#include <fstream>
#include <string>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

typedef pcl::PointXYZI PointType;

using namespace std;
using namespace Eigen;
double secs_init, nsecs_init, last_time;
bool is_img, is_init = false;
string bag_path, write_path, topic_name;

void parse_msg() 
{
    fstream file_;
    file_.open(bag_path, ios::in);
    if (!file_)
    {
        cout << "File " << bag_path << " does not exit" << endl;
        return;
    }
    
    rosbag::Bag bag;
    try
    {
        ROS_INFO("Start to load the rosbag %s", bag_path.c_str());
        bag.open(bag_path, rosbag::bagmode::Read);
        ROS_INFO("Bag %s opened", bag_path.c_str());
    }
    catch (rosbag::BagException e)
    {
        ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
        return;
    }

    vector<string> topics;
    if (is_img)
        topics.push_back(topic_name);
    else
        topics.push_back(string("/os_cloud_node/points"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    ofstream file_w;
    int cnt = 0;
    for (const rosbag::MessageInstance& m : view)
    {
        cout<<"process msg "<<cnt<<endl;
        double secs, nsecs, inc;
        if (is_img)
        {
            auto msg = *(m.instantiate<sensor_msgs::Image>());
            secs = msg.header.stamp.sec;
            nsecs = msg.header.stamp.nsec;
        }
        else
        {
            auto msg = *(m.instantiate<sensor_msgs::PointCloud2>());
            secs = msg.header.stamp.sec;
            nsecs = msg.header.stamp.nsec;
            if (!is_init)
            {
                secs_init = secs;
                nsecs_init = nsecs;
                is_init = true;
            }
            pcl::PointCloud<PointType>::Ptr pc(new pcl::PointCloud<PointType>);
            pcl::fromROSMsg(msg, *pc);
            double t = secs-secs_init + (nsecs-nsecs_init)*1e-9;
            file_w.open(write_path+to_string(t), std::ofstream::trunc);
            for (size_t i = 0; i < pc->points.size(); i++)
            {
                file_w << pc->points[i].x << "\t" << pc->points[i].y << "\t" << pc->points[i].z << "\t"
                    << pc->points[i].intensity << "\n";
            }
            file_w.close();
        }
        cnt++;
    }
    cout<<"complete"<<endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "parse_topic");
    ros::NodeHandle nh("~");

    nh.getParam("bag_path", bag_path);
    nh.getParam("write_path", write_path);
    nh.getParam("topic_name", topic_name);
    nh.getParam("is_img_topic", is_img);
    
    parse_msg();
    
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}