#ifndef _CAMSTABLIZER_HH_
#define _CAMSTABLIZER_HH_
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

// #include "std_msgs/Float32.h"
// #include "vans_copter/XYZ.h"
#include "vans_copter/RPY.h"
// #include "vans_copter/rpm.h"
#include "vans_copter/customPID.h"
#include "vans_copter/DualFilter.h"
#include "vans_copter/CamParam.h"
#include "vans_copter/CamCtrl.h"

// #include "vans_copter/Command.h"
// #include "vans_copter/PlanningPoint.h"
#include "darknet_ros_msgs/BoundingBoxes.h"


namespace gazebo
{
    // enum mode{grounded, planning, maneuverable};

    class camStablizer : public ModelPlugin
    {
    //node default varables
        private: 
        event::ConnectionPtr updateConnection;
        std::unique_ptr<ros::NodeHandle> rosNode;
        ros::CallbackQueue rosQueue;
        std::thread rosQueueThread;
        std::string linkName_roll,  linkName_pitch,  linkName_yaw;
        std::string jointName_roll, jointName_pitch, jointName_yaw;
        physics::ModelPtr model;
    //Cam varables
        ros::Subscriber rpySub, cmdSub, CamParamSub, CamCtrlSub;//planningSub
        vans_copter::RPY rpyInfo;
        vans_copter::CamParam CamParam;
        // vans_copter::Command receivedCmd;
        int frame;
        double previousTime, switchCamPitch, switchCamYaw;
        // double des_Yaw;//des_X, des_Y, des_Z, des_dYaw
    //PID varables
        int TBL;
        customPID dYaw_PID;
        customPID dRoll_PID;
        customPID dPitch_PID;
        // double dYaw_Pgain, dYaw_Igain, dYaw_Dgain;
        double dT_Pgain, dT_Igain, dT_Dgain, dT_Ilimit;
        double camTurnVel;
        // double YawVelLimit;
    //varables end
        public: camStablizer() {}

        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
        //_____________________________________________________ROS_node______________________________________________________________
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&camStablizer::OnUpdate, this));

            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, _model->GetName() + "_camStablizer_node",ros::init_options::NoSigintHandler);
            }
            this->rosNode.reset(new ros::NodeHandle(_model->GetName() + "_PLATFORM"));
        //Parameters for the link
            std::string modelTypeName;
            if(_sdf->HasElement("modelTypeName"))
            {
                modelTypeName=_sdf->Get<std::string>("modelTypeName");
                std::cerr << "Loading CamStablizer to: " <<_model->GetName()<< "...\n";
            }
            if(_sdf->HasElement("link_roll"))
            {
                this->linkName_roll=modelTypeName + "::" + (_sdf->Get<std::string>("link_roll"));
                this->jointName_roll=modelTypeName + "::joint_" + (_sdf->Get<std::string>("link_roll"));
            }
            if(_sdf->HasElement("link_pitch"))
            {
                this->linkName_pitch=modelTypeName + "::" + (_sdf->Get<std::string>("link_pitch"));
                this->jointName_pitch=modelTypeName + "::joint_" + (_sdf->Get<std::string>("link_pitch"));
            }
            if(_sdf->HasElement("link_yaw"))
            {
                this->linkName_yaw=modelTypeName + "::" + (_sdf->Get<std::string>("link_yaw"));
                this->jointName_yaw=modelTypeName + "::joint_" + (_sdf->Get<std::string>("link_yaw"));
            }
        //Parameters for Rolling movement
            if(_sdf->HasElement("TBL"))
                this->TBL=_sdf->Get<int>("TBL");
            // if(_sdf->HasElement("T_Pgain"))
                //     this->T_Pgain=_sdf->Get<double>("T_Pgain");
                // if(_sdf->HasElement("T_Igain"))
                //     this->T_Igain=_sdf->Get<double>("T_Igain");
                // if(_sdf->HasElement("T_Dgain"))
                //     this->T_Dgain=_sdf->Get<double>("T_Dgain");
                // if(_sdf->HasElement("T_Ilimit"))
            //     this->T_Ilimit=_sdf->Get<double>("T_Ilimit");
            if(_sdf->HasElement("dT_Pgain"))
                this->dT_Pgain=_sdf->Get<double>("dT_Pgain");
            if(_sdf->HasElement("dT_Igain"))
                this->dT_Igain=_sdf->Get<double>("dT_Igain");
            if(_sdf->HasElement("dT_Dgain"))
                this->dT_Dgain=_sdf->Get<double>("dT_Dgain");
            if(_sdf->HasElement("dT_Ilimit"))
                this->dT_Ilimit=_sdf->Get<double>("dT_Ilimit");
            if(_sdf->HasElement("camTurnVel"))
                this->camTurnVel=_sdf->Get<double>("camTurnVel");
            // if(_sdf->HasElement("T_Mgain"))
            //     this->T_Mgain=_sdf->Get<double>("T_Mgain");
        //Parameters for Yaw movement
            // if(_sdf->HasElement("YawBL"))
            //     this->YawBL=_sdf->Get<int>("YawBL");
            // if(_sdf->HasElement("Yaw_Pgain"))
            //     this->Yaw_Pgain=_sdf->Get<double>("Yaw_Pgain");
            // if(_sdf->HasElement("Yaw_Igain"))
            //     this->Yaw_Igain=_sdf->Get<double>("Yaw_Igain");
            // if(_sdf->HasElement("Yaw_Dgain"))
            //     this->Yaw_Dgain=_sdf->Get<double>("Yaw_Dgain");
            // if(_sdf->HasElement("Yaw_Ilimit"))
            //     this->Yaw_Ilimit=_sdf->Get<double>("Yaw_Ilimit");
            // if(_sdf->HasElement("dYaw_Pgain"))
            //     this->dYaw_Pgain=_sdf->Get<double>("dYaw_Pgain");
            // if(_sdf->HasElement("dYaw_Igain"))
            //     this->dYaw_Igain=_sdf->Get<double>("dYaw_Igain");
            // if(_sdf->HasElement("dYaw_Dgain"))
            //     this->dYaw_Dgain=_sdf->Get<double>("dYaw_Dgain");
            // if(_sdf->HasElement("dYaw_Ilimit"))
            //     this->dYaw_Ilimit=_sdf->Get<double>("dYaw_Ilimit");
            // if(_sdf->HasElement("Yaw_Mgain"))
            //     this->Yaw_Mgain=_sdf->Get<double>("Yaw_Mgain");
        //Parameters for limitation
            // if(_sdf->HasElement("RollLimit"))
            //     this->RollLimit=(_sdf->Get<double>("RollLimit"))/180.0*M_PI;
            // if(_sdf->HasElement("YawVelLimit"))
            //     this->YawVelLimit=(_sdf->Get<double>("YawVelLimit"))/180.0*M_PI;
        //Parameters for subscription & pulisher
            ros::SubscribeOptions rpySo =ros::SubscribeOptions::create<vans_copter::RPY>(_model->GetName()+"/Cam/filtRPY", 1, boost::bind(&camStablizer::rpyCallback, this, _1), ros::VoidPtr(), &this->rosQueue);
            ros::SubscribeOptions CamParamSo =ros::SubscribeOptions::create<vans_copter::CamParam>(_model->GetName()+"/CamParam", 1, boost::bind(&camStablizer::CamParamCallback, this, _1), ros::VoidPtr(), &this->rosQueue);
            ros::SubscribeOptions CamCtrlSo =ros::SubscribeOptions::create<vans_copter::CamCtrl>(_model->GetName()+"/CamCtrl", 1, boost::bind(&camStablizer::CamCtrlCallback, this, _1), ros::VoidPtr(), &this->rosQueue);

            // ros::SubscribeOptions cmdSo =ros::SubscribeOptions::create<vans_copter::Command>(_model->GetName()+"/KeycmdProcess", 1, boost::bind(&camStablizer::cmdProcessCallback, this, _1), ros::VoidPtr(), &this->rosQueue);
            this->rpySub = this->rosNode->subscribe(rpySo);
            this->CamParamSub = this->rosNode->subscribe(CamParamSo);
            this->CamCtrlSub = this->rosNode->subscribe(CamCtrlSo);

            // this->cmdSub = this->rosNode->subscribe(cmdSo);
            this->rosQueueThread =std::thread(std::bind(&camStablizer::QueueThread, this));
            this->model=_model;
        //Parameters for Com Initialization
            // ignition::math::Pose3d Pose=this->model->GetLink(this->linkNameFull)->WorldPose();
                // this->receivedCmd.Xvalue=Pose.Pos().X();
                // this->receivedCmd.Yvalue=Pose.Pos().Y();
                // this->receivedCmd.Zvalue=Pose.Pos().Z();
                // this->receivedCmd.dXvalue=0.0;
                // this->receivedCmd.dYvalue=0.0;
                // this->receivedCmd.dZvalue=0.0;
                // this->receivedCmd.dYawvalue=0.0;
                // this->receivedCmd.mode=vans_copter::Command::grid;
                // this->receivedCmd.modeSwitch=false;
                // this->receivedCmd.landed=true;
                // this->receivedCmd.hover=false;
                // this->receivedCmd.frame=0;
            // this->Yaw_PID=customPID(this->YawBL, this->Yaw_Ilimit);
            this->dYaw_PID=customPID(this->TBL, this->dT_Ilimit);
            // this->Roll_PID=customPID(this->TBL, this->T_Ilimit);
            this->dRoll_PID=customPID(this->TBL, this->dT_Ilimit);
            // this->Pitch_PID=customPID(this->TBL, this->T_Ilimit);
            this->dPitch_PID=customPID(this->TBL, this->dT_Ilimit);
            this->CamParam.pitch=0;
            this->CamParam.yaw=0;
            this->switchCamPitch=0;
            this->switchCamYaw=0;
        //End of Load function
        }
        public: void rpyCallback(const vans_copter::RPY::ConstPtr&_msg)
        {
            for(int n=0;n<3;n++)
            {
                this->rpyInfo.rpyAng[n]=_msg->rpyAng[n];
                this->rpyInfo.rpyAngVel[n]=_msg->rpyAngVel[n];
            }
        }
        public: void CamParamCallback(const vans_copter::CamParam::ConstPtr&_msg)
        {
            this->CamParam.pitch+=(_msg->pitch*0.2);
            this->CamParam.yaw+=(_msg->yaw*0.3);
            this->CamParam.pitch=ignition::math::clamp(this->CamParam.pitch, -0.4, 0.4);
            this->CamParam.yaw=ignition::math::clamp(this->CamParam.yaw, -0.6, 0.6);
            if(this->CamParam.pitch==0)
                this->switchCamPitch=this->rpyInfo.rpyAng[1];
            if(this->CamParam.yaw==0)
                this->switchCamYaw=this->rpyInfo.rpyAng[2];
        }
        public: void CamCtrlCallback(const vans_copter::CamCtrl::ConstPtr& msg)
        {
            if(msg->detected)
            {
                this->CamParam.pitch=-LinDeadzone(msg->posY_N, 0.03, 0.5);
                this->CamParam.yaw=-LinDeadzone(msg->posX_N, 0.03, 0.5);
            }
            else
            {
                this->CamParam.pitch=0;
                this->CamParam.yaw=0;
                this->switchCamPitch=this->rpyInfo.rpyAng[1];
                this->switchCamYaw=this->rpyInfo.rpyAng[2];
            }
        }
        // public: void cmdProcessCallback(const vans_copter::Command::ConstPtr&_msg)
            // {
            //     this->receivedCmd.Xvalue=_msg->Xvalue;
            //     this->receivedCmd.Yvalue=_msg->Yvalue;
            //     this->receivedCmd.Zvalue=_msg->Zvalue;
            //     this->receivedCmd.dXvalue=_msg->dXvalue;
            //     this->receivedCmd.dYvalue=_msg->dYvalue;
            //     this->receivedCmd.dZvalue=_msg->dZvalue;
            //     this->receivedCmd.dYawvalue=_msg->dYawvalue;
            //     this->receivedCmd.mode=_msg->mode;
            //     this->receivedCmd.modeSwitch=_msg->modeSwitch;
            //     this->receivedCmd.landed=_msg->landed;
            //     this->receivedCmd.hover=_msg->hover;
            //     this->receivedCmd.frame=_msg->frame;
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
                // PID_A(this->receivedCmd.Xvalue, this->receivedCmd.Yvalue, this->receivedCmd.Zvalue, deltaTime, currentTime);//输入为惯性系下的坐标值
                // if(this->receivedCmd.mode==vans_copter::Command::grid&&this->receivedCmd.modeSwitch==false)
                // {
                //     //grid模式下使用位置PID值
                //     this->receivedCmd.dXvalue=this->X_PID.Result();
                //     this->receivedCmd.dYvalue=this->Y_PID.Result();
                //     this->receivedCmd.dZvalue=this->Z_PID.Result();
                // }
                // PID_B(this->receivedCmd.dXvalue, this->receivedCmd.dYvalue, this->receivedCmd.dZvalue, deltaTime, currentTime);//输入为弹道系下的速度值
                            // PID_B(this->des_X, this->des_Y, this->des_Z, deltaTime, currentTime);//输入为弹道系下的速度值
                // double dYawSwitch=0.0;
                // if(this->receivedCmd.dYawvalue!=0)
                // {
                //     this->des_Yaw=this->rpyInfo.rpyAng[2];//进行偏航时记录更新PID偏航值
                //     dYawSwitch=this->receivedCmd.dYawvalue;
                // }
                // else if(this->receivedCmd.dYawvalue==0)
                // {
                //     dYawSwitch=this->Yaw_PID.Result();//无偏航角速度输入时利用最后一次记录的偏航角度值作固定
                // }
            // PID_C(0.0, deltaTime, currentTime);//输入为惯性系下的偏航角度值
            PID_D(deltaTime, currentTime);//输入为惯性系下的偏航角速度值
            SendSignalToMotors();
            // this->receivedCmdPub.publish(this->receivedCmd);
        }
        // public: void PID_A(double des_X, double des_Y, double des_Z, double dt, double ct)
            // {
            //     double X_err=des_X - this->xyzInfo.linPos[0];
            //     double Y_err=des_Y - this->xyzInfo.linPos[1];
            //     double Z_err=des_Z - this->xyzInfo.linPos[2];

            //     double X_err_remap=X_Yaw_remap(X_err, Y_err, this->rpyInfo.rpyAng[2]);
            //     double Y_err_remap=Y_Yaw_remap(X_err, Y_err, this->rpyInfo.rpyAng[2]);

            //     this->X_PID.Update(dt, ct, X_err_remap, this->Horiz_Pgain, this->Horiz_Igain, this->Horiz_Dgain);
            //     this->Y_PID.Update(dt, ct, Y_err_remap, this->Horiz_Pgain, this->Horiz_Igain, this->Horiz_Dgain);
            //     this->Z_PID.Update(dt, ct, Z_err,       this->H_Pgain,     this->H_Igain,     this->H_Dgain);
            // }
            // public: void PID_B(double des_dX, double des_dY, double des_dZ, double dt, double ct)
            // {
            //     double dX_err=ignition::math::clamp(
            //                 des_dX, -this->HorizontalVelLimit, this->HorizontalVelLimit)
            //                 -X_Yaw_remap(this->xyzInfo.linVel[0], this->xyzInfo.linVel[1], this->rpyInfo.rpyAng[2]);
            //     double dY_err=ignition::math::clamp(
            //                 des_dY, -this->HorizontalVelLimit, this->HorizontalVelLimit)
            //                 -Y_Yaw_remap(this->xyzInfo.linVel[0], this->xyzInfo.linVel[1], this->rpyInfo.rpyAng[2]);
            //     double dZ_err=ignition::math::clamp(
            //                 des_dZ, -this->VerticalVelLimit,   this->VerticalVelLimit)
            //                 -this->xyzInfo.linVel[2];

            //     this->dX_PID.Update(dt, ct, dX_err, this->dHoriz_Pgain, this->dHoriz_Igain, this->dHoriz_Dgain);
            //     this->dY_PID.Update(dt, ct, dY_err, this->dHoriz_Pgain, this->dHoriz_Igain, this->dHoriz_Dgain);
            //     this->dZ_PID.Update(dt, ct, dZ_err, this->dH_Pgain,     this->dH_Igain,     this->dH_Dgain);
            // }
            // public: void PID_C(double des_Yaw, double dt, double ct)
            // {
            //     double R_err=ignition::math::clamp(0.0, -this->RollLimit, this->RollLimit)
            //                 - this->rpyInfo.rpyAng[0];
            //     double P_err=ignition::math::clamp(0.0, -this->RollLimit, this->RollLimit)
            //                 - this->rpyInfo.rpyAng[1];
            //     double Yaw_err=0.0 - this->rpyInfo.rpyAng[2];

            //     this->Roll_PID.Update( dt, ct, R_err,   this->T_Pgain,   this->T_Igain,   this->T_Dgain);
            //     this->Pitch_PID.Update(dt, ct, P_err,   this->T_Pgain,   this->T_Igain,   this->T_Dgain);
            //     this->Yaw_PID.Update(  dt, ct, Yaw_err, this->Yaw_Pgain, this->Yaw_Igain, this->Yaw_Dgain);
        // }
        public: void PID_D(double dt, double ct)
        {
            // double dYaw_err=ignition::math::clamp(
            //                 this->Yaw_PID.Result(), -this->YawVelLimit, this->YawVelLimit) -this->rpyInfo.VYvalue;
            double dR_err=  0.0 - this->rpyInfo.rpyAng[0];
            double dP_err, dYaw_err;
            if(this->CamParam.pitch)
                dP_err=  this->CamParam.pitch;
            else
                dP_err=  this->switchCamPitch - this->rpyInfo.rpyAng[1];
            if(this->CamParam.yaw)
                dYaw_err=  this->CamParam.yaw;
            else
                dYaw_err=  this->switchCamYaw - this->rpyInfo.rpyAng[2];
            // std::cerr<<"err: "<<dR_err<<" "<<dP_err<<" "<<dYaw_err<<"\n";
            this->dRoll_PID.Update( dt, ct, dR_err,   this->dT_Pgain, this->dT_Igain, this->dT_Dgain);
            this->dPitch_PID.Update(dt, ct, dP_err,   this->dT_Pgain, this->dT_Igain, this->dT_Dgain);
            this->dYaw_PID.Update(  dt, ct, dYaw_err, this->dT_Pgain, this->dT_Igain, this->dT_Dgain);
        }
        public: void SendSignalToMotors()
        {
            // double U0=this->dZ_PID.Result()*    this->H_Mgain;
            double UR=ignition::math::clamp(this->dRoll_PID.Result(),  -this->camTurnVel, this->camTurnVel);
            double UP=ignition::math::clamp(this->dPitch_PID.Result(), -this->camTurnVel, this->camTurnVel);
            double UY=ignition::math::clamp(this->dYaw_PID.Result(),   -this->camTurnVel, this->camTurnVel);
            // double UR=dR_err*0.8;
            // double UP=dP_err*0.8;
            // double UY=dYaw_err*0.8;
            // if(dR_err>0)
            //     UR=0.8;
            // else if(dR_err<0)
            //     UR=-0.8;
            // else
            //     UR=0;
            // if(dP_err>0)
            //     UP=0.8;
            // else if(dP_err<0)
            //     UP=-0.8;
            // else
            //     UP=0;
            // if(dYaw_err>0)
            //     UY=0.8;
            // else if(dYaw_err<0)
            //     UY=-0.8;
            // else
            //     UY=0;
            // vans_copter::rpm rpm;
                // rpm.msgTime=ct;
                // double ratio=2*M_PI/60;
                // if(!this->receivedCmd.landed)
                // {
                //     rpm.radiensM[0]=ignition::math::clamp(U0 + U1 - U2 + U3, this->minRPM*ratio, this->maxRPM*ratio);
                //     rpm.radiensM[1]=ignition::math::clamp(U0 + U1 + U2 - U3, this->minRPM*ratio, this->maxRPM*ratio);
                //     rpm.radiensM[2]=ignition::math::clamp(U0 - U1 + U2 + U3, this->minRPM*ratio, this->maxRPM*ratio);
                //     rpm.radiensM[3]=ignition::math::clamp(U0 - U1 - U2 - U3, this->minRPM*ratio, this->maxRPM*ratio);
                // }
                // else
                // {
                //     rpm.radiensM[0]=0;
                //     rpm.radiensM[1]=0;
                //     rpm.radiensM[2]=0;
                //     rpm.radiensM[3]=0;
            // }
            // std::cerr<<"signal: "<<UR<<" "<<UP<<" "<<UY<<"\n";
            if(this->rpyInfo.rpyAng[1] < -0.7 && UP < 0)
                UP=0;
            else if(this->rpyInfo.rpyAng[1] > 1.2 && UP > 0)
                UP=0;

            this->model->GetJoint(this->jointName_roll)->SetVelocity(0,UR);
            this->model->GetJoint(this->jointName_pitch)->SetVelocity(0,UP);
            this->model->GetJoint(this->jointName_yaw)->SetVelocity(0,UY);
        }
        double LinDeadzone(double in, double front, double endtop)
        {
            double k=endtop/(1-front);
            double out;
            if(in <= -front)
            {
                out=k*(in+front);
            }
            else if(in>front)
            {
                out=k*(in-front);
            }
            else
            {
                out=0;
            }
            return out;
        }
    };
  GZ_REGISTER_MODEL_PLUGIN(camStablizer)
}
#endif