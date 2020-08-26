#include <ros/ros.h>
#include <iostream>
// #include <windows.h>
#include "std_msgs/Float32.h"
#include "vans_copter/PlanningPoint.h"

using namespace std;
int main(int argc,char **argv)
{
    ros::init(argc,argv,"publisher");
    ros::NodeHandle n;
    ros::Publisher pub=n.advertise<vans_copter::PlanningPoint>("/ROTORS_1st_PLATFORM/ROTORS_1st/publisher",1);
    ros::Rate loop_rate(2000);

    while (ros::ok())
    {
        ros::spinOnce();

        int num=0;
        vans_copter::PlanningPoint Points_colle;
        cout<<"^^^^^^^^^^^^^^ Route planning ^^^^^^^^^^^^^^\n";
        while (1)
        {
            printf("Add an anchor? [y/n] :");
            char Confirm;
            cin>>Confirm;
            if(Confirm=='n')
                break;
            else if(Confirm=='y')
            {
                cout<<"Please set XYZ for anchor["<<num<<"]\n";
                cout<<"X: ";
                cin>>Points_colle.PosX[num];
                cout<<"Y: ";
                cin>>Points_colle.PosY[num];
                cout<<"Z: ";
                cin>>Points_colle.PosZ[num];
                num++;
            }
        }
        if(num>=1)
        {
            Points_colle.PointNum=num;
            printf("Your inputs are:   X      Y      Z\n");
            for(int i=0;i<num;i++)
            {
                printf("  anchor[%02d]: %6.1f %6.1f %6.1f\n", i, Points_colle.PosX[i], Points_colle.PosY[i], Points_colle.PosZ[i]);
            }
            printf("Confirm? [y/n] :");
            char Confirm;
            cin>>Confirm;
            if(Confirm=='n')
            {
                printf("Setting canceled, continue planning? [y/n] :");
                char Confirm;
                cin>>Confirm;
                if(Confirm=='n')
                {
                    printf("Exit.\n");
                    return 0;
                }
                else if(Confirm=='y')
                    continue;
            }
            else if(Confirm=='y')
            {
                pub.publish(Points_colle);
                printf("Sending route... continue planning? [y/n] :");
                char Confirm;
                cin>>Confirm;
                if(Confirm=='n')
                {
                    printf("Exit.\n");
                    return 0;
                }
                else if(Confirm=='y')
                    continue;
            }
        }
        else
        {
            printf("No anchors have been set, continue planning? [y/n] :");
            char Confirm;
            cin>>Confirm;
            if(Confirm=='n')
            {
                printf("Exit.\n");
                return 0;
            }
            else if(Confirm=='y')
                continue;
        }
        loop_rate.sleep();
    }
}