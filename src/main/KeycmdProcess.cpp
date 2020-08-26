#ifndef _KEYCMDPROCESS_HH_
#define _KEYCMDPROCESS_HH_
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

#include "vans_copter/XYZ.h"
#include "vans_copter/RPY.h"
#include "vans_copter/customPID.h"
#include "vans_copter/DualFilter.h"
#include "vans_copter/Command.h"
#include "vans_copter/PlanningPoint.h"
#include "vans_copter/CamCtrl.h"
#include "vans_copter/CamParam.h"
#include "vans_copter/VecForward.h"


namespace gazebo
{
  enum mode{planning, maneuverable};

  class CmdProcessPlugin : public ModelPlugin
  {
    //node default varables
        private: 
        event::ConnectionPtr updateConnection;
        std::unique_ptr<ros::NodeHandle> rosNode;
        ros::CallbackQueue rosQueue;
        std::thread rosQueueThread;
        physics::ModelPtr model;
    //CmdProcess varables
        ros::Subscriber xyzSub, rpySub, cmdSub, planningSub, CamCtrlSub, CamRPYSub;
        ros::Publisher cmdProcessPub, PosPrePub;
        std::string linkNameFull;
        vans_copter::Command receivedCmd, sendCmd;
        vans_copter::XYZ xyzInfo;
        vans_copter::RPY rpyInfo, Camrpyinfo;
        vans_copter::PlanningPoint pointsColle;
        vans_copter::CamCtrl CamCtrl;
        customPID planeXPID, planeYPID, heightPID;
        int frame;
        int currentPlanningPointIndex;
        int mode;
        int isCamCtrl_ing;
        int CamCtrl_ingCount;
        double previousTime;
        double Pgain,Dgain,Vgain,avoidDist;
        double distTgtBuf[2][500];
        double ControlX[2][500];
        double ControlY[2][500];
        double ControlZ[2][500];

        
    public: CmdProcessPlugin() {}

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
    //_____________________________________________________ROS_Node______________________________________________________________
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&CmdProcessPlugin::OnUpdate, this));

        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, _model->GetName() + "CmdProcess_node",ros::init_options::NoSigintHandler);
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
            std::cerr << "Loading KeycmdProcess plugin to: " <<_model->GetName()<< "...\n";
        }
        if(_sdf->HasElement("Pgain"))
            this->Pgain=_sdf->Get<double>("Pgain");
        if(_sdf->HasElement("Dgain"))
            this->Dgain=_sdf->Get<double>("Dgain");
        if(_sdf->HasElement("Vgain"))
            this->Vgain=_sdf->Get<double>("Vgain");
        if(_sdf->HasElement("avoidDist"))
            this->avoidDist=_sdf->Get<double>("avoidDist");
    //Parameters for subscription & pulisher
        ros::SubscribeOptions xyzSo =ros::SubscribeOptions::create<vans_copter::XYZ>(               _model->GetName()+"/filtXYZ"     , 1, boost::bind(&CmdProcessPlugin::xyzCallback, this, _1), ros::VoidPtr(), &this->rosQueue);
        ros::SubscribeOptions rpySo =ros::SubscribeOptions::create<vans_copter::RPY>(               _model->GetName()+"/filtRPY"     , 1, boost::bind(&CmdProcessPlugin::rpyCallback, this, _1), ros::VoidPtr(), &this->rosQueue);
        ros::SubscribeOptions cmdSo =ros::SubscribeOptions::create<vans_copter::Command>(           _model->GetName()+"/Keycmd"      , 1, boost::bind(&CmdProcessPlugin::keyCmdCallback, this, _1),ros::VoidPtr(), &this->rosQueue);
        ros::SubscribeOptions CamRPYSo =ros::SubscribeOptions::create<vans_copter::RPY>(            _model->GetName()+"/Cam/filtRPY" , 1, boost::bind(&CmdProcessPlugin::CamRPYCallback, this, _1), ros::VoidPtr(), &this->rosQueue);
        ros::SubscribeOptions CamCtrlSo =ros::SubscribeOptions::create<vans_copter::CamCtrl>(       _model->GetName()+"/CamCtrl"     , 1, boost::bind(&CmdProcessPlugin::CamCtrlCallback, this, _1), ros::VoidPtr(), &this->rosQueue);
        ros::SubscribeOptions planningSo =ros::SubscribeOptions::create<vans_copter::PlanningPoint>(_model->GetName()+"/publisher"   , 1, boost::bind(&CmdProcessPlugin::planningCallback, this, _1), ros::VoidPtr(), &this->rosQueue);

        this->xyzSub = this->rosNode->subscribe(xyzSo);
        this->rpySub = this->rosNode->subscribe(rpySo);
        this->cmdSub = this->rosNode->subscribe(cmdSo);
        this->CamRPYSub = this->rosNode->subscribe(CamRPYSo);
        this->CamCtrlSub = this->rosNode->subscribe(CamCtrlSo);
        this->planningSub = this->rosNode->subscribe(planningSo);

        this->cmdProcessPub=this->rosNode->advertise<vans_copter::Command>(_model->GetName()+"/KeycmdProcess", 1);
        this->PosPrePub=this->rosNode->advertise<vans_copter::XYZ>(_model->GetName()+"/PosPrePub", 1);


        this->rosQueueThread =std::thread(std::bind(&CmdProcessPlugin::QueueThread, this));
        this->model=_model;
    //Parameters for KeycmdProcess Initialization
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
        this->currentPlanningPointIndex=0;
        this->mode=maneuverable;
        this->isCamCtrl_ing=false;
        this->CamCtrl_ingCount=1;
        this->planeXPID=customPID(50, 1.8);
        this->planeYPID=customPID(50, 1.8);
        this->heightPID=customPID(50, 1.8);
        for(int i=0;i<2;i++)
        {
            this->distTgtBuf[i][500]={};
            this->ControlX[i][500]={};
            this->ControlY[i][500]={};
            this->ControlZ[i][500]={};

        }
        //receivedCmd是用于接受控制指令的buff，而不会随插件自身更新而变化
        //sendCmd是发送到Com的指令，随插件自身更新而变化
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
    public: void CamRPYCallback(const vans_copter::RPY::ConstPtr&_msg)
    {
        this->Camrpyinfo=*_msg;
    }
    public: void CamCtrlCallback(const vans_copter::CamCtrl::ConstPtr &msg)
    {
        if(this->receivedCmd.mode!=vans_copter::Command::velocityEarth)
        {
            this->receivedCmd.mode=vans_copter::Command::velocityEarth;
            this->receivedCmd.modeSwitch=true;
        }
        this->isCamCtrl_ing=true;
        // std::cerr<<"mode: "<<this->receivedCmd.mode<<"\n";
        this->CamCtrl=*msg;
    }
    public: void keyCmdCallback(const vans_copter::Command::ConstPtr &_msg)
    {
        if(this->mode!=maneuverable)
        {
            this->receivedCmd.dXvalue=0;
            this->receivedCmd.dYvalue=0;
            this->receivedCmd.dZvalue=0;//速度初始化
            this->mode=maneuverable;
        }//外模式转换
        //receivedCmd为采集的指令,在mode==grid且modeswitch==true时无法更新采集指令
        if(!(this->receivedCmd.mode==vans_copter::Command::grid&&this->receivedCmd.modeSwitch==true))
        {
            this->receivedCmd.Xvalue+=_msg->Xvalue;
            this->receivedCmd.Yvalue+=_msg->Yvalue;
            this->receivedCmd.Zvalue+=_msg->Zvalue;
            this->receivedCmd.dXvalue+=(_msg->dXvalue*2.5);
            this->receivedCmd.dYvalue+=(_msg->dYvalue*2.5);
            this->receivedCmd.dZvalue+=(_msg->dZvalue*1.5);
            this->receivedCmd.dYawvalue+=(_msg->dYawvalue*0.35);
            this->receivedCmd.mode=_msg->mode;
            this->receivedCmd.modeSwitch=_msg->modeSwitch;
            this->receivedCmd.landed=_msg->landed;
            this->receivedCmd.hover=_msg->hover;
            this->receivedCmd.frame=_msg->frame;
            //限幅
            this->receivedCmd.dXvalue=ignition::math::clamp(receivedCmd.dXvalue, -5.0, 5.0);
            this->receivedCmd.dYvalue=ignition::math::clamp(receivedCmd.dYvalue, -5.0, 5.0);
            this->receivedCmd.dZvalue=ignition::math::clamp(receivedCmd.dZvalue, -3.0, 3.0);
            this->receivedCmd.dYawvalue=ignition::math::clamp(receivedCmd.dYawvalue, -0.7, 0.7);
        }
    }
    public: void planningCallback(const vans_copter::PlanningPoint::ConstPtr&_msg)
    {
        this->currentPlanningPointIndex=0;
        this->pointsColle.PointNum=_msg->PointNum;
        if(this->mode!=planning)
        {
            this->mode=planning;
            std::cerr<<this->model->GetName()<<" mode: planning... ";
            std::cerr<<"number of planningPoints: "<<_msg->PointNum<<"\n";
        }//外模式转换
        for(int i=0;i<_msg->PointNum;i++)
        {
            this->pointsColle.PosX[i]=_msg->PosX[i];
            this->pointsColle.PosY[i]=_msg->PosY[i];
            this->pointsColle.PosZ[i]=_msg->PosZ[i];
        }//转存
    }
    private: void QueueThread()
    {
        static const double timeout = 0.01;
        while (this->rosNode->ok())
        {
            this->rosQueue.callAvailable(ros::WallDuration(timeout));
        }
    }
    public: void TraceWithCamera(double dt, double ct)
    {
        if(this->isCamCtrl_ing)
        {
            ignition::math::Pose3d BodyPose=this->model->GetLink(this->linkNameFull)->WorldPose();
            ignition::math::Vector3d BodyForward(1, 0, 0);
            ignition::math::Vector3d BodyForward_G=BodyPose.Rot().RotateVector(BodyForward);
            ignition::math::Vector3d BodyForward_plane(BodyForward_G.X(), BodyForward_G.Y(),0);
            BodyForward_plane.Normalize();
            
            // ignition::math::Vector3d PictAngleBias;
            // double PictPitchBias= - this->CamCtrl.posY_N*20/180*M_PI;
            // double PictYawBias= - this->CamCtrl.posX_N*20/180*M_PI;
            // PictAngleBias.Set(0.0, PictPitchBias, PictYawBias);

            // ignition::math::Quaterniond PictBiasRot(PictAngleBias.X(), PictAngleBias.Y(), PictAngleBias.Z());
            ignition::math::Vector3d CamForward(1,0,0);
            // ignition::math::Vector3d CamForwardBiased=PictBiasRot*CamForward;
            ignition::math::Vector3d CamForwardBiased(1.0/tan(20.0/180.0*M_PI), -this->CamCtrl.posX_N, this->CamCtrl.posY_N);
            CamForwardBiased.Normalize();

            ignition::math::Vector3d CamrpyAngle;
            CamrpyAngle.Set(this->Camrpyinfo.rpyAng[0],this->Camrpyinfo.rpyAng[1],this->Camrpyinfo.rpyAng[2]);
            ignition::math::Quaterniond CamRot(CamrpyAngle.X(), CamrpyAngle.Y(), CamrpyAngle.Z());

            ignition::math::Vector3d Arrow2Tgt=CamRot.RotateVector(CamForwardBiased);
            Arrow2Tgt.Normalize();

            ignition::math::Vector3d CamForwardI=CamRot.RotateVector(CamForward);
            ignition::math::Vector3d CamForward_plane(CamForwardI.X(), CamForwardI.Y(), 0);
            CamForward_plane.Normalize();

            double cos_dephi=BodyForward_plane.Dot(CamForward_plane);
            ignition::math::Vector3d crossing=BodyForward_plane.Cross(CamForward_plane);
            double dephi;
            if(crossing.Z()>=0)
                dephi=acos(cos_dephi);
            else
                dephi=-acos(cos_dephi);

            double distPreFilt=dualFilter(this->distTgtBuf[0], this->distTgtBuf[1], 500, this->CamCtrl.distance_pre, 40.0);
            ignition::math::Vector3d dist2Tgt=Arrow2Tgt*distPreFilt;
            ignition::math::Vector3d dist2TgtPlane(dist2Tgt.X(), dist2Tgt.Y(), 0);
            double distPrePlane=dist2TgtPlane.Length();
            dist2TgtPlane.Normalize();
            ignition::math::Vector3d dist2TgtArea=dist2TgtPlane*(distPrePlane - this->avoidDist);

            double ControlXFilt=dualFilter(this->ControlX[0], this->ControlX[1], 100, dist2TgtArea.X()                , 38);
            double ControlYFilt=dualFilter(this->ControlY[0], this->ControlY[1], 100, dist2TgtArea.Y()                , 38);
            double ControlZFilt=dualFilter(this->ControlZ[0], this->ControlZ[1], 100, dist2Tgt.Z()+this->avoidDist*0, 38);



            this->planeXPID.Update(dt, ct, ControlXFilt, this->Pgain    , 0.15, this->Dgain);
            this->planeYPID.Update(dt, ct, ControlYFilt, this->Pgain    , 0.15, this->Dgain);
            this->heightPID.Update(dt, ct, ControlZFilt, this->Pgain*0.8, 0.15, this->Dgain);

                // ignition::math::Vector3d inputLimit=CamForward*3.5;
                // vans_copter::CamParam distPre;
                // distPre.pitch=this->CamCtrl.distance_pre;
                // distPre.yaw=distPreFilt;

                // this->distprePub.publish(distPre);
                // this->receivedCmd.dXvalue=ignition::math::clamp(this->planeXPID.Result(),-abs(inputLimit.X()),abs(inputLimit.X()));
                // this->receivedCmd.dYvalue=ignition::math::clamp(this->planeYPID.Result(),-abs(inputLimit.Y()),abs(inputLimit.Y()));
            // this->sendCmd.dZvalue=1*this->heightPID.Result();



            this->receivedCmd.dXvalue=ignition::math::clamp(this->planeXPID.Result()*this->Vgain, -5.0,5.0);
            this->receivedCmd.dYvalue=ignition::math::clamp(this->planeYPID.Result()*this->Vgain, -5.0,5.0);
            this->receivedCmd.dZvalue=ignition::math::clamp(this->heightPID.Result()*this->Vgain, -3.0,3.0);
            this->receivedCmd.dYawvalue=dephi*2;

            vans_copter::XYZ PosPre;
            PosPre.linPos[0]=this->xyzInfo.linPos[0]+dist2Tgt.X();
            PosPre.linPos[1]=this->xyzInfo.linPos[1]+dist2Tgt.Y();
            PosPre.linPos[2]=this->xyzInfo.linPos[2]+dist2Tgt.Z();

            PosPre.linVel[0]=this->xyzInfo.linVel[0];
            PosPre.linVel[1]=this->xyzInfo.linVel[1];
            PosPre.linVel[2]=this->xyzInfo.linVel[2];
            this->PosPrePub.publish(PosPre);

        }
    }
    public: void OnUpdate()
    {
        double deltaTime;
        double currentTime=this->model->GetWorld()->SimTime().Double();
        if(this->frame==0)
            deltaTime=1.0/2500.0;
        else
            deltaTime=currentTime-this->previousTime;
        this->previousTime=currentTime;
        this->frame++;
        if(this->mode==planning)
        {
            ignition::math::Vector3d posErr(this->pointsColle.PosX[this->currentPlanningPointIndex]-this->xyzInfo.linPos[0],
                                            this->pointsColle.PosY[this->currentPlanningPointIndex]-this->xyzInfo.linPos[1],
                                            this->pointsColle.PosZ[this->currentPlanningPointIndex]-this->xyzInfo.linPos[2]);//位置误差
            posErr.Correct();
            double velABS=5.0;//期望速度绝对值
            ignition::math::Vector3d targetVel=posErr/posErr.Length()*velABS;
            if(this->currentPlanningPointIndex<pointsColle.PointNum)
            {
                if(this->currentPlanningPointIndex<(pointsColle.PointNum-1))
                {
                    this->receivedCmd.mode=vans_copter::Command::velocityEarth;//内模式转换
                    this->receivedCmd.dXvalue=targetVel.X();
                    this->receivedCmd.dYvalue=targetVel.Y();
                    this->receivedCmd.dZvalue=targetVel.Z();//转存速度
                }
                else if(this->currentPlanningPointIndex==(pointsColle.PointNum-1))
                {
                    this->receivedCmd.mode=vans_copter::Command::grid;//内模式转换
                    this->receivedCmd.modeSwitch=false;
                    this->receivedCmd.Xvalue=this->pointsColle.PosX[this->currentPlanningPointIndex];
                    this->receivedCmd.Yvalue=this->pointsColle.PosY[this->currentPlanningPointIndex];
                    this->receivedCmd.Zvalue=this->pointsColle.PosZ[this->currentPlanningPointIndex];//到达终点前改为grid模式
                }
                if(posErr.Length()<0.5)
                {
                    std::cerr<<this->model->GetName()<<" reach PlanningPoint["<<this->currentPlanningPointIndex<<"]: "<<this->pointsColle.PosX[this->currentPlanningPointIndex]<<" "<<this->pointsColle.PosY[this->currentPlanningPointIndex]<<" "<<this->pointsColle.PosZ[this->currentPlanningPointIndex]<<"\n";
                    this->currentPlanningPointIndex++;
                }
            }
            if(this->currentPlanningPointIndex==pointsColle.PointNum)
                this->mode==maneuverable;//到达终点时计划模式更改为手动模式
        }
        if(!(this->receivedCmd.mode==vans_copter::Command::grid&&this->receivedCmd.modeSwitch==false))
        {
            //更新位置信息
            this->receivedCmd.Xvalue=this->xyzInfo.linPos[0];
            this->receivedCmd.Yvalue=this->xyzInfo.linPos[1];
            this->receivedCmd.Zvalue=this->xyzInfo.linPos[2];
        }
        else
        {
        }
        
        //填充控制指令
        this->sendCmd=this->receivedCmd;

        if(this->receivedCmd.mode==vans_copter::Command::velocityEarth)
        {
            TraceWithCamera(deltaTime, currentTime);
            //重映射到机体轴
            double dX_remap=X_Yaw_remap(this->receivedCmd.dXvalue, this->receivedCmd.dYvalue, this->rpyInfo.rpyAng[2]);
            double dY_remap=Y_Yaw_remap(this->receivedCmd.dXvalue, this->receivedCmd.dYvalue, this->rpyInfo.rpyAng[2]);
            this->sendCmd.dXvalue=dX_remap;
            this->sendCmd.dYvalue=dY_remap;
        }
        this->cmdProcessPub.publish(this->sendCmd);
        if(this->receivedCmd.modeSwitch)
        {
            if(this->isCamCtrl_ing)
            {
                if(this->CamCtrl_ingCount==0)
                    this->isCamCtrl_ing=false;
                else if(this->CamCtrl_ingCount==1)
                    this->CamCtrl_ingCount--;
            }
            //模式切换时速度置零
            this->receivedCmd.dXvalue=0;
            this->receivedCmd.dYvalue=0;
            this->receivedCmd.dZvalue=0;
            this->receivedCmd.dYawvalue=0;
            if(this->receivedCmd.mode!=vans_copter::Command::grid)
                this->receivedCmd.modeSwitch=false;//非grid模式开关量马上置零
            else
            {
                ignition::math::Vector3d vel(this->xyzInfo.linVel[0], this->xyzInfo.linVel[1], this->xyzInfo.linVel[2]);
                if(vel.Length()<=0.01)
                {
                    //速度小于0.01后开关量置零
                    this->receivedCmd.modeSwitch=false;//表示切换完成，速度归零，进入grid模式，这时才可以再次接受控制
                }
            }
        }
        if(this->receivedCmd.landed)
        {
            //landed时速度置零
            this->receivedCmd.dXvalue=0;
            this->receivedCmd.dYvalue=0;
            this->receivedCmd.dZvalue=0;
            this->receivedCmd.dYawvalue=0;
        }
    }
  };
  GZ_REGISTER_MODEL_PLUGIN(CmdProcessPlugin)
}
#endif