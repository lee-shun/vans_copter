#ifndef _COM2_HH_
#define _COM2_HH_
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

#include "std_msgs/Float32.h"
#include "vans_copter/XYZ.h"
#include "vans_copter/RPY.h"
#include "vans_copter/rpm.h"
#include "vans_copter/customPID.h"
#include "vans_copter/DualFilter.h"
#include "vans_copter/Command.h"
#include "vans_copter/PlanningPoint.h"

namespace gazebo
{
    // enum mode{grounded, planning, maneuverable};

    class ComPlugin : public ModelPlugin
    {
    //node default varables
        private: 
        event::ConnectionPtr updateConnection;
        std::unique_ptr<ros::NodeHandle> rosNode;
        ros::CallbackQueue rosQueue;
        std::thread rosQueueThread;
        std::string linkNameFull;
        physics::ModelPtr model;
    //Com varables
        ros::Subscriber xyzSub, rpySub, cmdSub;//planningSub
        ros::Publisher rosPub, receivedCmdPub;
        vans_copter::XYZ xyzInfo;
        vans_copter::RPY rpyInfo;
        vans_copter::Command receivedCmd;
        // vans_copter::PlanningPoint pointsColle;
        int frame;
        // int mode;
        // int currentPlanningPointIndex;
        double previousTime;
        double des_Yaw;//des_X, des_Y, des_Z, des_dYaw

    //PID varables
        int YawBL, HBL, TBL, HorizBL;
        customPID Z_PID, dZ_PID, Yaw_PID,   dYaw_PID;
        customPID Y_PID, dY_PID, Roll_PID,  dRoll_PID;
        customPID X_PID, dX_PID, Pitch_PID, dPitch_PID;
        double Horiz_Pgain, Horiz_Igain, Horiz_Dgain, dHoriz_Pgain, dHoriz_Igain, dHoriz_Dgain;
        double Yaw_Pgain, Yaw_Igain, Yaw_Dgain, dYaw_Pgain, dYaw_Igain, dYaw_Dgain, Yaw_Mgain;
        double H_Pgain,   H_Igain,   H_Dgain,   dH_Pgain,   dH_Igain,   dH_Dgain,   H_Mgain;
        double T_Pgain,   T_Igain,   T_Dgain,   dT_Pgain,   dT_Igain,   dT_Dgain,   T_Mgain;
        double minRPM,    maxRPM;
        double VerticalVelLimit,     HorizontalVelLimit,    YawVelLimit,            RollLimit;
        double H_Ilimit, dH_Ilimit, Yaw_Ilimit, dYaw_Ilimit;
        double Horiz_Ilimit, dHoriz_Ilimit, T_Ilimit, dT_Ilimit;
    //varables end
        public: ComPlugin() {}

        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
        //_____________________________________________________ROS_node______________________________________________________________
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&ComPlugin::OnUpdate, this));

            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, _model->GetName() + "_Com_node",ros::init_options::NoSigintHandler);
            }
            this->rosNode.reset(new ros::NodeHandle(_model->GetName() + "_PLATFORM"));
        //Parameters for the link
            std::string modelTypeName;
            if(_sdf->HasElement("modelTypeName"))
            {
                modelTypeName=_sdf->Get<std::string>("modelTypeName");
            }
            if(_sdf->HasElement("link"))
            {
                this->linkNameFull=modelTypeName + "::" + (_sdf->Get<std::string>("link"));
                std::cerr << "Loading Com plugin to: " <<_model->GetName()<< "...\n";
            }
        //Parameters for Z movement
            if(_sdf->HasElement("HBL"))
                this->HBL=_sdf->Get<int>("HBL");
            if(_sdf->HasElement("H_Pgain"))
                this->H_Pgain=_sdf->Get<double>("H_Pgain");
            if(_sdf->HasElement("H_Igain"))
                this->H_Igain=_sdf->Get<double>("H_Igain");
            if(_sdf->HasElement("H_Dgain"))
                this->H_Dgain=_sdf->Get<double>("H_Dgain");
            if(_sdf->HasElement("H_Ilimit"))
                this->H_Ilimit=_sdf->Get<double>("H_Ilimit");
            if(_sdf->HasElement("dH_Pgain"))
                this->dH_Pgain=_sdf->Get<double>("dH_Pgain");
            if(_sdf->HasElement("dH_Igain"))
                this->dH_Igain=_sdf->Get<double>("dH_Igain");
            if(_sdf->HasElement("dH_Dgain"))
                this->dH_Dgain=_sdf->Get<double>("dH_Dgain");
            if(_sdf->HasElement("dH_Ilimit"))
                this->dH_Ilimit=_sdf->Get<double>("dH_Ilimit");
            if(_sdf->HasElement("H_Mgain"))
                this->H_Mgain=_sdf->Get<double>("H_Mgain");
        //Parameters for Horizontal movement
            if(_sdf->HasElement("HorizBL"))
                this->HorizBL=_sdf->Get<int>("HorizBL");
            if(_sdf->HasElement("Horiz_Pgain"))
                this->Horiz_Pgain=_sdf->Get<double>("Horiz_Pgain");
            if(_sdf->HasElement("Horiz_Igain"))
                this->Horiz_Igain=_sdf->Get<double>("Horiz_Igain");
            if(_sdf->HasElement("Horiz_Dgain"))
                this->Horiz_Dgain=_sdf->Get<double>("Horiz_Dgain");
            if(_sdf->HasElement("Horiz_Ilimit"))
                this->Horiz_Ilimit=_sdf->Get<double>("Horiz_Ilimit");
            if(_sdf->HasElement("dHoriz_Pgain"))
                this->dHoriz_Pgain=_sdf->Get<double>("dHoriz_Pgain");
            if(_sdf->HasElement("dHoriz_Igain"))
                this->dHoriz_Igain=_sdf->Get<double>("dHoriz_Igain");
            if(_sdf->HasElement("dHoriz_Dgain"))
                this->dHoriz_Dgain=_sdf->Get<double>("dHoriz_Dgain");
            if(_sdf->HasElement("dHoriz_Ilimit"))
                this->dHoriz_Ilimit=_sdf->Get<double>("dHoriz_Ilimit");
        //Parameters for Rolling movement
            if(_sdf->HasElement("TBL"))
                this->TBL=_sdf->Get<int>("TBL");
            if(_sdf->HasElement("T_Pgain"))
                this->T_Pgain=_sdf->Get<double>("T_Pgain");
            if(_sdf->HasElement("T_Igain"))
                this->T_Igain=_sdf->Get<double>("T_Igain");
            if(_sdf->HasElement("T_Dgain"))
                this->T_Dgain=_sdf->Get<double>("T_Dgain");
            if(_sdf->HasElement("T_Ilimit"))
                this->T_Ilimit=_sdf->Get<double>("T_Ilimit");
            if(_sdf->HasElement("dT_Pgain"))
                this->dT_Pgain=_sdf->Get<double>("dT_Pgain");
            if(_sdf->HasElement("dT_Igain"))
                this->dT_Igain=_sdf->Get<double>("dT_Igain");
            if(_sdf->HasElement("dT_Dgain"))
                this->dT_Dgain=_sdf->Get<double>("dT_Dgain");
            if(_sdf->HasElement("dT_Ilimit"))
                this->dT_Ilimit=_sdf->Get<double>("dT_Ilimit");
            if(_sdf->HasElement("T_Mgain"))
                this->T_Mgain=_sdf->Get<double>("T_Mgain");
        //Parameters for Yaw movement
            if(_sdf->HasElement("YawBL"))
                this->YawBL=_sdf->Get<int>("YawBL");
            if(_sdf->HasElement("Yaw_Pgain"))
                this->Yaw_Pgain=_sdf->Get<double>("Yaw_Pgain");
            if(_sdf->HasElement("Yaw_Igain"))
                this->Yaw_Igain=_sdf->Get<double>("Yaw_Igain");
            if(_sdf->HasElement("Yaw_Dgain"))
                this->Yaw_Dgain=_sdf->Get<double>("Yaw_Dgain");
            if(_sdf->HasElement("Yaw_Ilimit"))
                this->Yaw_Ilimit=_sdf->Get<double>("Yaw_Ilimit");
            if(_sdf->HasElement("dYaw_Pgain"))
                this->dYaw_Pgain=_sdf->Get<double>("dYaw_Pgain");
            if(_sdf->HasElement("dYaw_Igain"))
                this->dYaw_Igain=_sdf->Get<double>("dYaw_Igain");
            if(_sdf->HasElement("dYaw_Dgain"))
                this->dYaw_Dgain=_sdf->Get<double>("dYaw_Dgain");
            if(_sdf->HasElement("dYaw_Ilimit"))
                this->dYaw_Ilimit=_sdf->Get<double>("dYaw_Ilimit");
            if(_sdf->HasElement("Yaw_Mgain"))
                this->Yaw_Mgain=_sdf->Get<double>("Yaw_Mgain");
        //Parameters for limitation
            if(_sdf->HasElement("minRPM"))
                this->minRPM=_sdf->Get<double>("minRPM");
            if(_sdf->HasElement("maxRPM"))
                this->maxRPM=_sdf->Get<double>("maxRPM");
            if(_sdf->HasElement("VerticalVelLimit"))
                this->VerticalVelLimit=_sdf->Get<double>("VerticalVelLimit");
            if(_sdf->HasElement("HorizontalVelLimit"))
                this->HorizontalVelLimit=_sdf->Get<double>("HorizontalVelLimit");
            if(_sdf->HasElement("RollLimit"))
                this->RollLimit=(_sdf->Get<double>("RollLimit"))/180.0*M_PI;
            if(_sdf->HasElement("YawVelLimit"))
                this->YawVelLimit=(_sdf->Get<double>("YawVelLimit"))/180.0*M_PI;
        //Parameters for subscription & pulisher
            ros::SubscribeOptions xyzSo =ros::SubscribeOptions::create<vans_copter::XYZ>(_model->GetName()+"/filtXYZ", 1, boost::bind(&ComPlugin::xyzCallback, this, _1), ros::VoidPtr(), &this->rosQueue);
            ros::SubscribeOptions rpySo =ros::SubscribeOptions::create<vans_copter::RPY>(_model->GetName()+"/filtRPY", 1, boost::bind(&ComPlugin::rpyCallback, this, _1), ros::VoidPtr(), &this->rosQueue);
            ros::SubscribeOptions cmdSo =ros::SubscribeOptions::create<vans_copter::Command>(_model->GetName()+"/KeycmdProcess", 1, boost::bind(&ComPlugin::cmdProcessCallback, this, _1), ros::VoidPtr(), &this->rosQueue);
            // ros::SubscribeOptions planningSo =ros::SubscribeOptions::create<vans_copter::PlanningPoint>("/publisher", 1, boost::bind(&ComPlugin::planningCallback, this, _1), ros::VoidPtr(), &this->rosQueue);

            this->xyzSub = this->rosNode->subscribe(xyzSo);
            this->rpySub = this->rosNode->subscribe(rpySo);
            this->cmdSub = this->rosNode->subscribe(cmdSo);
            // this->planningSub = this->rosNode->subscribe(planningSo);

            this->rosPub=this->rosNode->advertise<vans_copter::rpm>(_model->GetName()+"/Com", 1);
            this->receivedCmdPub=this->rosNode->advertise<vans_copter::Command>(_model->GetName()+"/Com/receivedCmd", 1);
            this->rosQueueThread =std::thread(std::bind(&ComPlugin::QueueThread, this));
            this->model=_model;
        //Parameters for Com Initialization
            ignition::math::Pose3d Pose=this->model->GetLink(this->linkNameFull)->WorldPose();
            this->receivedCmd.Xvalue=Pose.Pos().X();
            this->receivedCmd.Yvalue=Pose.Pos().Y();
            this->receivedCmd.Zvalue=Pose.Pos().Z();
            this->receivedCmd.dXvalue=0.0;
            this->receivedCmd.dYvalue=0.0;
            this->receivedCmd.dZvalue=0.0;
            this->receivedCmd.dYawvalue=0.0;
            this->receivedCmd.mode=vans_copter::Command::grid;
            this->receivedCmd.modeSwitch=false;
            this->receivedCmd.landed=true;
            this->receivedCmd.hover=false;
            this->receivedCmd.frame=0;

            // this->currentPlanningPointIndex=0;

            this->Z_PID=customPID(this->HBL, this->H_Ilimit);
            this->dZ_PID=customPID(this->HBL, this->dH_Ilimit);
            this->Yaw_PID=customPID(this->YawBL, this->Yaw_Ilimit);
            this->dYaw_PID=customPID(this->YawBL, this->dYaw_Ilimit);

            this->Y_PID=customPID(this->HorizBL, this->Horiz_Ilimit);
            this->dY_PID=customPID(this->HorizBL, this->dHoriz_Ilimit);
            this->X_PID=customPID(this->HorizBL, this->Horiz_Ilimit);
            this->dX_PID=customPID(this->HorizBL, this->dHoriz_Ilimit);
            this->Roll_PID=customPID(this->TBL, this->T_Ilimit);
            this->dRoll_PID=customPID(this->TBL, this->dT_Ilimit);
            this->Pitch_PID=customPID(this->TBL, this->T_Ilimit);
            this->dPitch_PID=customPID(this->TBL, this->dT_Ilimit);
        //End of Load function
        }
        public: void xyzCallback(const vans_copter::XYZ::ConstPtr&_msg)
        {
            for(int n=0;n<3;n++)
            {
                this->xyzInfo.linPos[n]=_msg->linPos[n];
                this->xyzInfo.linVel[n]=_msg->linVel[n];
            }
        }
        public: void rpyCallback(const vans_copter::RPY::ConstPtr&_msg)
        {
            for(int n=0;n<3;n++)
            {
                this->rpyInfo.rpyAng[n]=_msg->rpyAng[n];
                this->rpyInfo.rpyAngVel[n]=_msg->rpyAngVel[n];
            }
        }
        public: void cmdProcessCallback(const vans_copter::Command::ConstPtr&_msg)
        {
            this->receivedCmd.Xvalue=_msg->Xvalue;
            this->receivedCmd.Yvalue=_msg->Yvalue;
            this->receivedCmd.Zvalue=_msg->Zvalue;
            this->receivedCmd.dXvalue=_msg->dXvalue;
            this->receivedCmd.dYvalue=_msg->dYvalue;
            this->receivedCmd.dZvalue=_msg->dZvalue;
            this->receivedCmd.dYawvalue=_msg->dYawvalue;
            this->receivedCmd.mode=_msg->mode;
            this->receivedCmd.modeSwitch=_msg->modeSwitch;
            this->receivedCmd.landed=_msg->landed;
            this->receivedCmd.hover=_msg->hover;
            this->receivedCmd.frame=_msg->frame;
        }
        // public: void planningCallback(const vans_copter::PlanningPoint::ConstPtr&_msg)
        // {
        //     this->currentPlanningPointIndex=0;
        //     this->pointsColle.PointNum=_msg->PointNum;
        //     if(this->mode!=planning)
        //     {
        //         this->mode=planning;
        //         std::cerr<<this->model->GetName()<<" mode: planning... ";
        //         std::cerr<<"number of planningPoints: "<<_msg->PointNum<<"\n";
        //     }
        //     for(int i=0;i<_msg->PointNum;i++)
        //     {
        //         this->pointsColle.PosX[i]=_msg->PosX[i];
        //         this->pointsColle.PosY[i]=_msg->PosY[i];
        //         this->pointsColle.PosZ[i]=_msg->PosZ[i];
        //     }
        // }
        private: void QueueThread()
        {
            static const double timeout = 0.01;
            while (this->rosNode->ok())
            {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }
        public: void OnUpdate()
        {
        //计算时间
            double deltaTime;
            double currentTime=this->model->GetWorld()->SimTime().Double();
            if(this->frame==0)
                deltaTime=1.0/2500.0;
            else
                deltaTime=currentTime-this->previousTime;
            this->previousTime=currentTime;
            this->frame++;
        //
            // PlanningModeCheck();
            PID_A(this->receivedCmd.Xvalue, this->receivedCmd.Yvalue, this->receivedCmd.Zvalue, deltaTime, currentTime);//输入为惯性系下的坐标值
            if(this->receivedCmd.mode==vans_copter::Command::grid&&this->receivedCmd.modeSwitch==false)
            {
                //grid模式下使用位置PID值
                this->receivedCmd.dXvalue=this->X_PID.Result();
                this->receivedCmd.dYvalue=this->Y_PID.Result();
                this->receivedCmd.dZvalue=this->Z_PID.Result();
            }
            PID_B(this->receivedCmd.dXvalue, this->receivedCmd.dYvalue, this->receivedCmd.dZvalue, deltaTime, currentTime);//输入为弹道系下的速度值
                        // PID_B(this->des_X, this->des_Y, this->des_Z, deltaTime, currentTime);//输入为弹道系下的速度值
            double dYawSwitch=0.0;
            if(this->receivedCmd.dYawvalue!=0)
            {
                this->des_Yaw=this->rpyInfo.rpyAng[2];//进行偏航时记录更新PID偏航值
                dYawSwitch=this->receivedCmd.dYawvalue;
            }
            else if(this->receivedCmd.dYawvalue==0)
            {
                dYawSwitch=this->Yaw_PID.Result();//无偏航角速度输入时利用最后一次记录的偏航角度值作固定
            }
            PID_C(this->des_Yaw, deltaTime, currentTime);//输入为惯性系下的偏航角度值
            PID_D(dYawSwitch, deltaTime, currentTime);//输入为惯性系下的偏航角速度值
            SendSignalToMotors(currentTime);
            this->receivedCmdPub.publish(this->receivedCmd);
        }
        // public: void PlanningModeCheck()
        // {
        //     if(this->mode==planning)
        //     {//执行计划模式
        //         if(this->currentPlanningPointIndex!=this->pointsColle.PointNum)
        //         {
        //             this->des_X=this->pointsColle.PosX[this->currentPlanningPointIndex];
        //             this->des_Y=this->pointsColle.PosY[this->currentPlanningPointIndex];
        //             this->des_Z=this->pointsColle.PosZ[this->currentPlanningPointIndex];
        //             double deltaX=this->des_X-this->xyzInfo.linPos[0];
        //             double deltaY=this->des_Y-this->xyzInfo.linPos[1];
        //             double deltaZ=this->des_Z-this->xyzInfo.linPos[2];
        //             double dist2nextPoint=sqrt(pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2));
        //             if(dist2nextPoint<0.25)
        //             {
        //                 std::cerr<<this->model->GetName()<<" reach PlanningPoint["<<this->currentPlanningPointIndex<<"]: "<<this->des_X<<" "<<this->des_Y<<" "<<this->des_Z<<"\n";
        //                 this->currentPlanningPointIndex++;
        //             }
        //         }
        //     }
        // }
        public: void PID_A(double des_X, double des_Y, double des_Z, double dt, double ct)
        {
            double X_err=des_X - this->xyzInfo.linPos[0];
            double Y_err=des_Y - this->xyzInfo.linPos[1];
            double Z_err=des_Z - this->xyzInfo.linPos[2];

            double X_err_remap=X_Yaw_remap(X_err, Y_err, this->rpyInfo.rpyAng[2]);
            double Y_err_remap=Y_Yaw_remap(X_err, Y_err, this->rpyInfo.rpyAng[2]);

            this->X_PID.Update(dt, ct, X_err_remap, this->Horiz_Pgain, this->Horiz_Igain, this->Horiz_Dgain);
            this->Y_PID.Update(dt, ct, Y_err_remap, this->Horiz_Pgain, this->Horiz_Igain, this->Horiz_Dgain);
            this->Z_PID.Update(dt, ct, Z_err,       this->H_Pgain,     this->H_Igain,     this->H_Dgain);
        }
        public: void PID_B(double des_dX, double des_dY, double des_dZ, double dt, double ct)
        {
            double dX_err=ignition::math::clamp(
                        des_dX, -this->HorizontalVelLimit, this->HorizontalVelLimit)
                        -X_Yaw_remap(this->xyzInfo.linVel[0], this->xyzInfo.linVel[1], this->rpyInfo.rpyAng[2]);
            double dY_err=ignition::math::clamp(
                        des_dY, -this->HorizontalVelLimit, this->HorizontalVelLimit)
                        -Y_Yaw_remap(this->xyzInfo.linVel[0], this->xyzInfo.linVel[1], this->rpyInfo.rpyAng[2]);
            double dZ_err=ignition::math::clamp(
                        des_dZ, -this->VerticalVelLimit,   this->VerticalVelLimit)
                        -this->xyzInfo.linVel[2];

            this->dX_PID.Update(dt, ct, dX_err, this->dHoriz_Pgain, this->dHoriz_Igain, this->dHoriz_Dgain);
            this->dY_PID.Update(dt, ct, dY_err, this->dHoriz_Pgain, this->dHoriz_Igain, this->dHoriz_Dgain);
            this->dZ_PID.Update(dt, ct, dZ_err, this->dH_Pgain,     this->dH_Igain,     this->dH_Dgain);
        }
        public: void PID_C(double des_Yaw, double dt, double ct)
        {
            double R_err=ignition::math::clamp( -this->dY_PID.Result(), -this->RollLimit, this->RollLimit)
                        - this->rpyInfo.rpyAng[0];
            double P_err=ignition::math::clamp(  this->dX_PID.Result(), -this->RollLimit, this->RollLimit)
                        - this->rpyInfo.rpyAng[1];
            double Yaw_err=des_Yaw - this->rpyInfo.rpyAng[2];

            this->Roll_PID.Update( dt, ct, R_err,   this->T_Pgain,   this->T_Igain,   this->T_Dgain);
            this->Pitch_PID.Update(dt, ct, P_err,   this->T_Pgain,   this->T_Igain,   this->T_Dgain);
            this->Yaw_PID.Update(  dt, ct, Yaw_err, this->Yaw_Pgain, this->Yaw_Igain, this->Yaw_Dgain);
        }
        public: void PID_D(double des_dYaw, double dt, double ct)
        {
            double dR_err=  this->Roll_PID.Result() -  this->rpyInfo.rpyAngVel[0];
            double dP_err=  this->Pitch_PID.Result() - this->rpyInfo.rpyAngVel[1];
            double dYaw_err=des_dYaw -this->rpyInfo.rpyAngVel[2];
            // double dYaw_err=ignition::math::clamp(
            //                 this->Yaw_PID.Result(), -this->YawVelLimit, this->YawVelLimit) -this->rpyInfo.VYvalue;
            this->dRoll_PID.Update( dt, ct, dR_err,   this->dT_Pgain,   this->dT_Igain,   this->dT_Dgain);
            this->dPitch_PID.Update(dt, ct, dP_err,   this->dT_Pgain,   this->dT_Igain,   this->dT_Dgain);
            this->dYaw_PID.Update(  dt, ct, dYaw_err, this->dYaw_Pgain, this->dYaw_Igain, this->dYaw_Dgain);
        }
        public: void SendSignalToMotors(double ct)
        {
            double U0=this->dZ_PID.Result()*    this->H_Mgain;
            double U1=this->dRoll_PID.Result()* this->T_Mgain;
            double U2=this->dPitch_PID.Result()*this->T_Mgain;
            double U3=this->dYaw_PID.Result()*  this->Yaw_Mgain;
            vans_copter::rpm rpm;
            rpm.msgTime=ct;
            double ratio=2*M_PI/60;
            if(!this->receivedCmd.landed)
            {
                rpm.radiensM[0]=ignition::math::clamp(U0 + U1 - U2 + U3, this->minRPM*ratio, this->maxRPM*ratio);
                rpm.radiensM[1]=ignition::math::clamp(U0 + U1 + U2 - U3, this->minRPM*ratio, this->maxRPM*ratio);
                rpm.radiensM[2]=ignition::math::clamp(U0 - U1 + U2 + U3, this->minRPM*ratio, this->maxRPM*ratio);
                rpm.radiensM[3]=ignition::math::clamp(U0 - U1 - U2 - U3, this->minRPM*ratio, this->maxRPM*ratio);
            }
            else
            {
                rpm.radiensM[0]=0;
                rpm.radiensM[1]=0;
                rpm.radiensM[2]=0;
                rpm.radiensM[3]=0;
            }
            this->rosPub.publish(rpm);
        }
    };
  GZ_REGISTER_MODEL_PLUGIN(ComPlugin)
}
#endif