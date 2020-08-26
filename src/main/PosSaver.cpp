#include <fstream>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <math.h>

#include "vans_copter/XYZ.h"

using namespace std;

class PosRecorder
{
    ros::NodeHandle nh_;
    ros::Subscriber MyPosSub, TgtPosSub, PosPreSub;
    vans_copter::XYZ MyPos, TgtPos, PosPre;
    ofstream of;

    public:
    PosRecorder()
    {
        MyPosSub  = nh_.subscribe<vans_copter::XYZ>("/ROTORS_1st_PLATFORM/ROTORS_1st/filtXYZ", 1, &PosRecorder::MyPosCB,  this);
        TgtPosSub = nh_.subscribe<vans_copter::XYZ>("/ROTORS_2st_PLATFORM/ROTORS_2st/filtXYZ", 1, &PosRecorder::TgtPosCB, this);
        PosPreSub = nh_.subscribe<vans_copter::XYZ>("/ROTORS_1st_PLATFORM/ROTORS_1st/PosPrePub", 1, &PosRecorder::PosPreCB, this);

        of.open("PosRecord.txt");
    }
    ~PosRecorder()
    {
        of.close();
    }
    void MyPosCB(const vans_copter::XYZ::ConstPtr &msg)
    {
        MyPos=*msg;
        Save2Pos();
    }
    void TgtPosCB(const vans_copter::XYZ::ConstPtr &msg)
    {
        TgtPos=*msg;
    }
    void PosPreCB(const vans_copter::XYZ::ConstPtr &msg)
    {
        PosPre=*msg;
    }
    void Save2Pos()
    {
        if(fmod(MyPos.msgTime, 0.25) < 0.0001)
        {
            of<<MyPos.msgTime<<" ";
            of<<MyPos.linPos[0]<<" ";
            of<<MyPos.linPos[1]<<" ";
            of<<MyPos.linPos[2]<<" ";
            of<<TgtPos.linPos[0]<<" ";
            of<<TgtPos.linPos[1]<<" ";
            of<<TgtPos.linPos[2]<<" ";
            of<<PosPre.linPos[0]<<" ";
            of<<PosPre.linPos[1]<<" ";//pospre
            of<<PosPre.linPos[2]<<" ";
            of<<PosPre.linVel[0]<<" ";
            of<<PosPre.linVel[1]<<" ";//vel
            of<<PosPre.linVel[2]<<"\n";
        }
    }
};

int main(int argc,char **argv)
{
	// int a[10];
        // ifstream in("读入.txt");
        // for(int i=0;i<10;i++)
        // 	in>>a[i];
        // for(int i=0;i<10;i++)
        // 	cout<<a[i]<<endl;
        // ofstream out("输出.txt");
        // for(int i=0;i<20;i++)
        // {
        //     out<<"ojbk "<<i<<" ojbk\n";
        // }
        // in.close();
        // out.close();
	// cin.get();
    ros::init(argc,argv,"PosSaver");
    PosRecorder PR;
    ros::spin();
}
