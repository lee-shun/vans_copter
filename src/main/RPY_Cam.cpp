#ifndef _RPY_CAM_HH_
#define _RPY_CAM_HH_
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include "ros/ros.h"
#include "vans_copter/RPY.h"
#include "vans_copter/DualFilter.h"
#include "vans_copter/VecForward.h"


namespace gazebo
{
  class RPYCam : public ModelPlugin
  {
    //node default varables
        private: 
        event::ConnectionPtr updateConnection;
        std::unique_ptr<ros::NodeHandle> rosNode;
        physics::ModelPtr model;
    //XYZ varables
        std::string linkname_full;
        ros::Publisher rawRPYPub, filtRPYPub, CamVecPub;
        int frame, VBL, ABL;
        double Vsigma, Asigma;
        double angVelBuf[2][3][1000], angAccBuf[2][3][1000];
        double previousTime;

    public: RPYCam() {}

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
    //____________________________________________________________ROS_node___________________________________________________________
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&RPYCam::OnUpdate, this));

        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, _model->GetName() + "_IMU_Cam_node",ros::init_options::NoSigintHandler);
        }
        this->rosNode.reset(new ros::NodeHandle(_model->GetName() + "_PLATFORM"));
    //Parameters for the filter
        std::string modelTypeName;
        if(_sdf->HasElement("modelTypeName"))
        {
            modelTypeName=_sdf->Get<std::string>("modelTypeName");
        }
        if(_sdf->HasElement("link"))
        {
            this->linkname_full=modelTypeName + "::" + (_sdf->Get<std::string>("link"));
            std::cerr << "Loading RPY_Cam plugin to: " <<_model->GetName()<< "...\n";
        }
        if(_sdf->HasElement("VBL"))
        {
            this->VBL=_sdf->Get<int>("VBL");
        }
        if(_sdf->HasElement("Vsigma"))
        {
            this->Vsigma=_sdf->Get<double>("Vsigma");
        }
        if(_sdf->HasElement("ABL"))
        {
            this->ABL=_sdf->Get<int>("ABL");
        }
        if(_sdf->HasElement("Asigma"))
        {
            this->Asigma=_sdf->Get<double>("Asigma");
        }
    //filter Initialization
        this->filtRPYPub=this->rosNode->advertise<vans_copter::RPY>(_model->GetName()+"/Cam/filtRPY", 1);
        this->CamVecPub=this->rosNode->advertise<vans_copter::VecForward>(_model->GetName()+"/CamVecForward",1);
        this->model=_model;
        for(int m=0;m<2;m++)
        {
            for(int n=0;n<3;n++)
            {
                this->angVelBuf[m][n][this->VBL]={};
                this->angAccBuf[m][n][this->ABL]={};
            }
        }
        this->previousTime=0.0;
    //End of Load function
    }
    public: void OnUpdate()
    {
        ignition::math::Pose3d   Pose  =this->model->GetLink(this->linkname_full)->WorldPose();
        ignition::math::Vector3d absAngVel=this->model->GetLink(this->linkname_full)->WorldAngularVel();
        ignition::math::Vector3d relAngVel=this->model->GetLink(this->linkname_full)->RelativeAngularVel();
        ignition::math::Vector3d absAngAcc=this->model->GetLink(this->linkname_full)->WorldAngularAccel();
        ignition::math::Vector3d relAngAcc=this->model->GetLink(this->linkname_full)->RelativeAngularAccel();
        
        double currentTime=this->model->GetWorld()->SimTime().Double();
        vans_copter::RPY rawRPY, filtRPY;

        filtRPY.rpyAng[0]=Pose.Rot().Roll();
        filtRPY.rpyAng[1]=Pose.Rot().Pitch();
        filtRPY.rpyAng[2]=Pose.Rot().Yaw();

        filtRPY.rpyAngVel[0]=dualFilter(this->angVelBuf[0][0], this->angVelBuf[1][0], this->VBL, relAngVel.X(), this->Vsigma);
        filtRPY.rpyAngVel[1]=dualFilter(this->angVelBuf[0][1], this->angVelBuf[1][1], this->VBL, relAngVel.Y(), this->Vsigma);
        filtRPY.rpyAngVel[2]=dualFilter(this->angVelBuf[0][2], this->angVelBuf[1][2], this->VBL, absAngVel.Z(), this->Vsigma);
        
        filtRPY.rpyAngAcc[0]=dualFilter(this->angAccBuf[0][0], this->angAccBuf[1][0], this->ABL, relAngAcc.X(), this->Asigma);
        filtRPY.rpyAngAcc[1]=dualFilter(this->angAccBuf[0][1], this->angAccBuf[1][1], this->ABL, relAngAcc.Y(), this->Asigma);
        filtRPY.rpyAngAcc[2]=dualFilter(this->angAccBuf[0][2], this->angAccBuf[1][2], this->ABL, absAngAcc.Z(), this->Asigma);
        
        filtRPY.msgTime=currentTime;

        this->filtRPYPub.publish(filtRPY);

        ignition::math::Vector3d CamForward(1,0,0);
        ignition::math::Vector3d CamForward_G=Pose.Rot().RotateVector(CamForward);

        vans_copter::VecForward CamVecForward;
        CamVecForward.VecForward[0]=CamForward_G.X();
        CamVecForward.VecForward[1]=CamForward_G.Y();
        CamVecForward.VecForward[2]=CamForward_G.Z();

        this->CamVecPub.publish(CamVecForward);
    }
  };
  GZ_REGISTER_MODEL_PLUGIN(RPYCam)
}
#endif