#ifndef _XYZ_PLGN_HH_
#define _XYZ_PLGN_HH_
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include "ros/ros.h"
#include "vans_copter/XYZ.h"
#include "vans_copter/DualFilter.h"

namespace gazebo
{
  class XYZPlugin : public ModelPlugin
  {
    //node default varables
        private: 
        event::ConnectionPtr updateConnection;
        std::unique_ptr<ros::NodeHandle> rosNode;
        physics::ModelPtr model;
    //XYZ varables
        std::string linkNameFull;
        ros::Publisher rawXYZPub, filtXYZPub;
        int frame, PBL, VBL, ABL;
        double Psigma, Vsigma, Asigma;
        double linVelBuf[2][3][1000], linAccBuf[2][3][1000];
        double previousTime;
    //varables end
    public: XYZPlugin() {}

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
    //____________________________________________________________ROS_node___________________________________________________________
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&XYZPlugin::OnUpdate, this));

        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, _model->GetName() + "XYZ_node",ros::init_options::NoSigintHandler);
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
            this->linkNameFull=modelTypeName + "::" + (_sdf->Get<std::string>("link"));
            std::cerr << "Loading XYZ plugin to: " <<_model->GetName()<< "...\n";
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
        this->filtXYZPub=this->rosNode->advertise<vans_copter::XYZ>(_model->GetName()+"/filtXYZ", 1);
        this->model=_model;
        for(int m=0;m<2;m++)
        {
            for(int n=0;n<3;n++)
            {
                this->linVelBuf[m][n][this->VBL]={};
                this->linAccBuf[m][n][this->ABL]={};
            }
        }
        this->previousTime=0.0;
    //End of Load function
    }
    public: void OnUpdate()
    {
        ignition::math::Pose3d   Pose  =this->model->GetLink(this->linkNameFull)->WorldPose();
        ignition::math::Vector3d LinVel=this->model->GetLink(this->linkNameFull)->WorldLinearVel();
        ignition::math::Vector3d LinAcc=this->model->GetLink(this->linkNameFull)->WorldLinearAccel();

        double CurrentTime=this->model->GetWorld()->SimTime().Double();
        vans_copter::XYZ rawXYZ, filtXYZ;

        filtXYZ.linPos[0]=Pose.Pos().X();
        filtXYZ.linPos[1]=Pose.Pos().Y();
        filtXYZ.linPos[2]=Pose.Pos().Z();

        filtXYZ.linVel[0]=dualFilter(this->linVelBuf[0][0], this->linVelBuf[1][0], this->VBL, LinVel.X(), this->Vsigma);
        filtXYZ.linVel[1]=dualFilter(this->linVelBuf[0][1], this->linVelBuf[1][1], this->VBL, LinVel.Y(), this->Vsigma);
        filtXYZ.linVel[2]=dualFilter(this->linVelBuf[0][2], this->linVelBuf[1][2], this->VBL, LinVel.Z(), this->Vsigma);
        
        filtXYZ.linAcc[0]=dualFilter(this->linAccBuf[0][0], this->linAccBuf[1][0], this->ABL, LinAcc.X(), this->Asigma);
        filtXYZ.linAcc[1]=dualFilter(this->linAccBuf[0][1], this->linAccBuf[1][1], this->ABL, LinAcc.Y(), this->Asigma);
        filtXYZ.linAcc[2]=dualFilter(this->linAccBuf[0][2], this->linAccBuf[1][2], this->ABL, LinAcc.Z(), this->Asigma);

        filtXYZ.msgTime=CurrentTime;

        this->filtXYZPub.publish(filtXYZ);
    }
  };
  GZ_REGISTER_MODEL_PLUGIN(XYZPlugin)
}
#endif