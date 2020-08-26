#include <iostream>
#include <ros/ros.h>
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "vans_copter/CamParam.h"
#include "vans_copter/CamCtrl.h"
static vans_copter::CamCtrl CamCtrl;

void bboxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
    int BBoxCount=msg->bounding_boxes.size();
    int recordIndex;
    double maxProbability=0;
    for(int i=0;i<BBoxCount;i++)
    {
        if(msg->bounding_boxes[i].probability>maxProbability)
        {
            maxProbability=msg->bounding_boxes[i].probability;
            recordIndex=i;
        }
    }
    if(BBoxCount>0)
    {
        std::cerr<<"UAV detected: "<<maxProbability<<" index: "<<recordIndex<<"\n";
        double bboxWidth=msg->bounding_boxes[recordIndex].xmax-msg->bounding_boxes[recordIndex].xmin;
        double bboxheight=msg->bounding_boxes[recordIndex].ymax-msg->bounding_boxes[recordIndex].ymin;
        double crossing=sqrt(bboxWidth*bboxWidth+bboxheight*bboxheight);
        double posX=(msg->bounding_boxes[recordIndex].xmax+msg->bounding_boxes[recordIndex].xmin)/2-208;
        double posY=(msg->bounding_boxes[recordIndex].ymax+msg->bounding_boxes[recordIndex].ymin)/2-208;
        double posX_N=posX/208;
        double posY_N=-posY/208;
        double distance_pre=5*117/crossing;
        std::cerr<<"posX_N: "<<posX_N<<" posY_N: "<<posY_N<<" distance_pre: "<<distance_pre<<"\n";
        CamCtrl.posX_N=posX_N;
        CamCtrl.posY_N=posY_N;
        CamCtrl.distance_pre=distance_pre;
        CamCtrl.detected=1;
    }
    // if(CamCtrl.detected==0)
    // {
    //     std::cerr<<"nothing detected~\n";
    // }
}
int main(int argc,char **argv)
{
    ros::init(argc,argv,"BBoxInfo");
    ros::NodeHandle n;
    ros::Subscriber BBoxSub;
    ros::Publisher  BBoxPub;
    BBoxSub=n.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes",1,bboxCallback);
    BBoxPub=n.advertise<vans_copter::CamCtrl>("/ROTORS_1st_PLATFORM/ROTORS_1st/CamCtrl",1);

    CamCtrl.detected=0;
    ros::Rate loop_rate(60);
    while(ros::ok())
    {
        ros::spinOnce();
        BBoxPub.publish(CamCtrl);
        CamCtrl.detected=0;
        loop_rate.sleep();
    }
}