#ifndef _MOTORPLGN_HH_
#define _MOTORPLGN_HH_
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "vans_copter/rpm.h"
#include <thread>

namespace gazebo {
class MotorPlugin : public ModelPlugin {
  //
private:
  event::ConnectionPtr updateConnection;
  std::unique_ptr<ros::NodeHandle> rosNode;
  ros::Subscriber rosSub;
  ros::CallbackQueue rosQueue;
  std::thread rosQueueThread;
  physics::ModelPtr model;
  //
  std::string bodyname_full;
  std::string motorname_full[4];
  std::string jointname_full[4];
  double mag[4];
  double dist2cp;
  double cl;
  double cd;
  double area;
  double rand;
  int blade_count;
  //
public:
  MotorPlugin() {}

public:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    //_______________________________________________________ROS__node_________________________________________________________________
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&MotorPlugin::OnUpdate, this));

    if (!ros::isInitialized()) {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, _model->GetName() + "_Motor_node",
                ros::init_options::NoSigintHandler);
    }
    this->rosNode.reset(new ros::NodeHandle(_model->GetName() + "_PLATFORM"));
    //_________________________________________________________________________________________________________________________________
    /**
      * 获得linkname以及joint_name
      *
      */
    std::string modelTypeName;
    if (_sdf->HasElement("modelTypeName")) {
      modelTypeName = _sdf->Get<std::string>("modelTypeName");
    }
    if (_sdf->HasElement("bodylink")) {
      this->bodyname_full =
          modelTypeName + "::" + (_sdf->Get<std::string>("bodylink"));
      std::cerr << "Loading Motor plugin to: " << _model->GetName() << "...\n";
    }
    if (_sdf->HasElement("motorlink0")) {
      this->motorname_full[0] =
          modelTypeName + "::" + (_sdf->Get<std::string>("motorlink0"));
      this->jointname_full[0] =
          modelTypeName + "::joint_" + (_sdf->Get<std::string>("motorlink0"));
    }
    if (_sdf->HasElement("motorlink1")) {
      this->motorname_full[1] =
          modelTypeName + "::" + (_sdf->Get<std::string>("motorlink1"));
      this->jointname_full[1] =
          modelTypeName + "::joint_" + (_sdf->Get<std::string>("motorlink1"));
    }
    if (_sdf->HasElement("motorlink2")) {
      this->motorname_full[2] =
          modelTypeName + "::" + (_sdf->Get<std::string>("motorlink2"));
      this->jointname_full[2] =
          modelTypeName + "::joint_" + (_sdf->Get<std::string>("motorlink2"));
    }
    if (_sdf->HasElement("motorlink3")) {
      this->motorname_full[3] =
          modelTypeName + "::" + (_sdf->Get<std::string>("motorlink3"));
      this->jointname_full[3] =
          modelTypeName + "::joint_" + (_sdf->Get<std::string>("motorlink3"));
    }
    //
    if (_sdf->HasElement("dist2cp"))
      this->dist2cp = _sdf->Get<double>("dist2cp");
    if (_sdf->HasElement("blade_count"))
      this->blade_count = _sdf->Get<int>("blade_count");
    if (_sdf->HasElement("cl"))
      this->cl = _sdf->Get<double>("cl");
    if (_sdf->HasElement("cd"))
      this->cd = _sdf->Get<double>("cd");
    if (_sdf->HasElement("area"))
      this->area = _sdf->Get<double>("area");
    if (_sdf->HasElement("rand"))
      this->rand = _sdf->Get<double>("rand");
    //
    //TODO:不熟悉的ros操作
    ros::SubscribeOptions so = ros::SubscribeOptions::create<vans_copter::rpm>(
        _model->GetName() + "/Com", 1,
        boost::bind(&MotorPlugin::FromComCallback, this, _1), ros::VoidPtr(),
        &this->rosQueue);

    rosSub = rosNode->subscribe(so);

    this->rosQueueThread =
        std::thread(std::bind(&MotorPlugin::QueueThread, this));

    this->model = _model;

    for (int i = 0; i < 4; i++) {
      this->mag[i] = 0.0;
    }
    //
  }

public:
  void FromComCallback(const vans_copter::rpm::ConstPtr &_msg) {
    for (int i = 0; i < 4; i++) {
      this->mag[i] = _msg->radiensM[i];
    }
  }

private:
  void QueueThread() {
    static const double timeout = 0.01;
    while (this->rosNode->ok()) {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

public:
  void OnUpdate() {
    ignition::math::Vector3d upward(0, 0, 1);
    ignition::math::Pose3d pose =
        this->model->GetLink(this->bodyname_full)->WorldPose();
    ignition::math::Vector3d upwardI = pose.Rot().RotateVector(upward);
    ignition::math::Vector3d lift[4];
    ignition::math::Vector3d torque[4];
    for (int i = 0; i < 4; i++) {
      double oscillation;
      if (this->mag[i] == 0)
        oscillation = 0;
      else
        oscillation =
            ignition::math::Rand::DblUniform(-this->rand, this->rand) *
            (this->mag[i] / 210.0);
      this->mag[i] += oscillation;
      double q = 0.5 * 1.293 * (this->mag[i] * this->mag[i]) *
                 (this->dist2cp * this->dist2cp);
      lift[i] = this->blade_count * this->cl * q * this->area * upwardI *
                (abs(this->mag[i]) / this->mag[i]); // mag是转速 弧度每秒
      torque[i] = this->blade_count * this->cd * q * this->dist2cp *
                  this->area * upwardI * (abs(this->mag[i]) / this->mag[i]);
      if (i == 1) // 1、3号正转产生负力矩
        torque[i] = -1.0 * torque[i];
      if (i == 3) // 1、3号正转产生负力矩
        torque[i] = -1.0 * torque[i];
      if (this->mag[i] == 0.0) // check转速
      {
        lift[i] = upwardI * 0.0;
        torque[i] = upwardI * 0.0;
      }
      lift[i].Correct();
      torque[i].Correct();
    }
    for (int i = 0; i < 4; i++) {

      this->model->GetJoint(this->jointname_full[i])
          ->SetVelocity(0, this->mag[i]);
      this->model->GetLink(this->motorname_full[i])->SetForce(lift[i]);
      this->model->GetLink(this->bodyname_full)->AddTorque(torque[i]);
    }
  }
};
GZ_REGISTER_MODEL_PLUGIN(MotorPlugin)
} // namespace gazebo
#endif
