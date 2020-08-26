#include <termio.h>
#include <ros/ros.h>
#include <ignition/math.hh>
#include "vans_copter/Command.h"
#include "vans_copter/CamParam.h"

//______________________________________________temp_keyboard_input________________________________________________________________
int getch(void)
{
     struct termios tm, tm_old;
     int fd = 0, ch;
     if (tcgetattr(fd, &tm) < 0) {//保存现在的终端设置
          return -1;
     }
     tm_old = tm;
     cfmakeraw(&tm);//更改终端设置为原始模式，该模式下所有的输入数据以字节为单位被处理
     if (tcsetattr(fd, TCSANOW, &tm) < 0) {//设置上更改之后的设置
          return -1;
     }
     ch = getchar();
     if (tcsetattr(fd, TCSANOW, &tm_old) < 0) {//更改设置为最初的样子
          return -1;
     }
     return ch;
}
using namespace std;

int main(int argc,char **argv)
{
    ros::init(argc,argv,"TargetCtrl");
    ros::NodeHandle n;
    ros::Publisher keycmdPub=n.advertise<vans_copter::Command>("/ROTORS_2st_PLATFORM/ROTORS_2st/Keycmd",1);
    ros::Publisher CamParamPub=n.advertise<vans_copter::CamParam>("/ROTORS_2st_PLATFORM/ROTORS_2st/CamParam",1);
    ros::Rate loop_rate(100);

    char a;
    int frame=0;
    vans_copter::Command cmd;
    vans_copter::CamParam CamParam;
    printf("|^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^|\n");
    printf("| Press Keys to transform   up & down   |  control mode    landing    |\n");
    printf("|           W                   F       |        M        (shutdown)  |\n");
    printf("|        A  S  D                V       |                     N       |\n");
    printf("|^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^|  grid                       |\n");
    printf("|      Yaw Rotation        end control  |  velocityEarth              |\n");
    printf("|        Q     E                R       |  velocityBody               |\n");
    printf(" ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ \n");
    cmd.mode=vans_copter::Command::grid;
    cmd.modeSwitch=false;
    cmd.landed=true;
    cmd.hover=false;
    printf("mode: grid\n");

    while (ros::ok())
    {
        cmd.Xvalue=0.0;
        cmd.Yvalue=0.0;
        cmd.Zvalue=0.0;
        cmd.dXvalue=0.0;
        cmd.dYvalue=0.0;
        cmd.dZvalue=0.0;
        cmd.dYawvalue=0.0;
        cmd.frame=frame;
        frame++;

        CamParam.pitch=0.0;
        CamParam.yaw=0.0;
        a=getch();
        if(a=='m')
        {
            cmd.modeSwitch=true;
            if(cmd.mode==vans_copter::Command::grid)
            {
                cmd.mode=vans_copter::Command::velocityEarth;
                printf("mode switched to: velocityEarth\n");
            }
            else if(cmd.mode==vans_copter::Command::velocityEarth)
            {
                cmd.mode=vans_copter::Command::velocityBody;
                printf("mode switched to: velocityBody\n");
            }
            else if(cmd.mode==vans_copter::Command::velocityBody)
            {
                cmd.mode=vans_copter::Command::grid;
                printf("mode switched to: grid\n");
            }
            cmd.landed=false;
            keycmdPub.publish(cmd);//发出模式切换指令
            cmd.modeSwitch=false;
        }
        else if(a=='r')
        {
            std::cerr<<"End keyboard control.\n";
            return 0;
        }
        else
        {
            if(a=='n')
            {
                if(cmd.mode!=vans_copter::Command::velocityEarth)
                {
                    cmd.mode=vans_copter::Command::velocityEarth;
                    cmd.modeSwitch=true;
                }
                cmd.landed=true;
                cmd.hover=false;
                keycmdPub.publish(cmd);
                cmd.modeSwitch=false;
                printf("landed.   mode switched to: velocityEarth\n");
            }
            else if(a=='i'||a=='k'||a=='j'||a=='l')
            {
                if(a=='i')
                    CamParam.pitch=-1;
                if(a=='k')
                    CamParam.pitch=1;
                if(a=='j')
                    CamParam.yaw=1;
                if(a=='l')
                    CamParam.yaw=-1;
                CamParamPub.publish(CamParam);
            }
            else
            {
                if(cmd.mode==vans_copter::Command::grid)
                {    
                    if (a=='w')
                        cmd.Xvalue=1.0;
                    if (a=='s')
                        cmd.Xvalue=-1.0;
                    if (a=='a')
                        cmd.Yvalue=1.0;
                    if (a=='d')
                        cmd.Yvalue=-1.0;
                    if (a=='f')
                        cmd.Zvalue=1.0;
                    if (a=='v')
                        cmd.Zvalue=-1.0;
                }
                else
                {    
                    if (a=='w')
                        cmd.dXvalue=1.0;
                    if (a=='s')
                        cmd.dXvalue=-1.0;
                    if (a=='a')
                        cmd.dYvalue=1.0;
                    if (a=='d')
                        cmd.dYvalue=-1.0;
                    if (a=='f')
                        cmd.dZvalue=1.0;
                    if (a=='v')
                        cmd.dZvalue=-1.0;
                }
                if(a=='q')
                    cmd.dYawvalue=1.0;
                if(a=='e')
                    cmd.dYawvalue=-1.0;
                cmd.landed=false;
                cmd.hover=false;
                keycmdPub.publish(cmd);
            }
        }
        
        loop_rate.sleep();
    }
}