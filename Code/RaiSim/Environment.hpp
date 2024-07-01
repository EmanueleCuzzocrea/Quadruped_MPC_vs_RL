#pragma once

#include <stdlib.h>
#include <set>
#include "../../RaisimGymEnv.hpp"
#include <iostream>
#include <random>
#include <chrono>
#include <iostream>
#include <fstream>

namespace raisim {

class ENVIRONMENT : public RaisimGymEnv {

 public:

  explicit ENVIRONMENT(const std::string& resourceDir, const Yaml::Node& cfg, bool visualizable) :
  
  RaisimGymEnv(resourceDir, cfg), visualizable_(visualizable), normDist_(0, 1) {
    /// create world
    world_ = std::make_unique<raisim::World>();

    /// add objects
    anymal_ = world_->addArticulatedSystem(resourceDir_+"/aliengo/aliengo.urdf");
    anymal_->setName("anymal");
    anymal_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    world_->addGround();

    /* ADD IRREGULAR TERRAIN
    raisim::TerrainProperties terrainProperties;
    terrainProperties.frequency = 50;
    terrainProperties.zScale = 0.05;
    terrainProperties.xSize = 10.0;
    terrainProperties.ySize = 10.0;
    terrainProperties.xSamples = 200;
    terrainProperties.ySamples = 200;
    terrainProperties.fractalOctaves = 3;
    terrainProperties.fractalLacunarity = 2.0;
    terrainProperties.fractalGain = 0.25;
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937 gen(seed);
    std::uniform_real_distribution<> dis(0, 99);
    double seed_ = dis(gen);
    terrainProperties.seed = seed_;
    //hm = world_->addHeightMap(10.0, 0.0, terrainProperties);
    */

    /// get robot data
    gcDim_ = anymal_->getGeneralizedCoordinateDim();
    gvDim_ = anymal_->getDOF();
    nJoints_ = gvDim_ - 6;

    /// initialize containers
    gc_.setZero(gcDim_); gc_init_.setZero(gcDim_);
    gv_.setZero(gvDim_); gv_init_.setZero(gvDim_); delta = 0; foot1_index = 0; foot2_index = 0; foot3_index = 0; foot4_index = 0; P_1 = 0; P_2 = 0; P_3 = 0; P_4 = 0;
    pTarget_.setZero(gcDim_); vTarget_.setZero(gvDim_); pTarget12_.setZero(nJoints_); contatti.setZero(4); q_trot = 0; q_bound = 0; q_pace = 0;
    kc = 0.1; yaw = 0;
    lastAction_.setZero(12); lastlastAction_.setZero(12);

    /// this is nominal configuration of anymal
    gc_init_ << 0, 0, 0.41, 1.0, 0.0, 0.0, 0.0, 0.06, 0.65, -1.4, -0.06, 0.65, -1.4, 0.06, 0.65, -1.4, -0.06, 0.65, -1.4;

    /// set pd gains
    Eigen::VectorXd jointPgain(gvDim_), jointDgain(gvDim_);
    jointPgain.setZero(); jointPgain.tail(nJoints_).setConstant(50.0);
    jointDgain.setZero(); jointDgain.tail(nJoints_).setConstant(0.2);
    anymal_->setPdGains(jointPgain, jointDgain);
    anymal_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));

    /// MUST BE DONE FOR ALL ENVIRONMENTS
    obDim_ = 69;
    actionDim_ = nJoints_; actionMean_.setZero(actionDim_); actionStd_.setZero(actionDim_);
    obDouble_.setZero(obDim_);

    /// action scaling
    actionMean_ = gc_init_.tail(nJoints_);
    double action_std;
    READ_YAML(double, action_std, cfg_["action_std"]) /// example of reading params from the config
    actionStd_.setConstant(action_std);

    /// Reward coefficients
    rewards_.initializeFromConfigurationFile (cfg["reward"]);

    /// indices of links that should not make contact with ground
    footIndices_.insert(anymal_->getBodyIdx("FL_calf"));
    footIndices_.insert(anymal_->getBodyIdx("FR_calf"));
    footIndices_.insert(anymal_->getBodyIdx("RR_calf"));
    footIndices_.insert(anymal_->getBodyIdx("RL_calf"));

    // Body index of each foot (for Reward Machines)
    foot1_index = 3;
    foot2_index = 6;
    foot3_index = 9;
    foot4_index = 12;

    /// visualize if it is the first environment
    if (visualizable_) {
      server_ = std::make_unique<raisim::RaisimServer>(world_.get());
      server_->launchServer();
      server_->focusOn(anymal_);
    }
  }


  void init() final { }


  void updateObservation() {
    anymal_->getState(gc_, gv_);
    raisim::Vec<4> quat;
    raisim::Mat<3,3> rot;
    quat[0] = gc_[3]; quat[1] = gc_[4]; quat[2] = gc_[5]; quat[3] = gc_[6];
    raisim::quatToRotMat(quat, rot);
    bodyLinearVel_ = rot.e().transpose() * gv_.segment(0, 3);
    bodyAngularVel_ = rot.e().transpose() * gv_.segment(3, 3);

    auto& contact = anymal_->getContacts();

    // Extraction of the yaw angle form a quaternion
    double siny_cosp = 2.0 * (quat[0] * quat[3] + quat[1] * quat[2]);
    double cosy_cosp = 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]);
    yaw = std::atan2(siny_cosp, cosy_cosp);

    obDouble_ <<  gc_[2],                           /// body height
                  rot.e().row(2).transpose(),       /// body orientation
                  gc_.tail(12),                     /// joint angles
                  bodyLinearVel_, bodyAngularVel_,  /// body linear&angular velocity
                  gv_.tail(12),                     /// joint velocity
                  P_1,                              /// 0 = stance foot, 1 = swing foot
                  P_2,                              /// 0 = stance foot, 1 = swing foot
                  P_3,                              /// 0 = stance foot, 1 = swing foot
                  P_4,                              /// 0 = stance foot, 1 = swing foot
                  q_trot,                           /// q0 or q1 (trot)
                  q_bound,                          /// q0 or q1 (bound)
                  q_pace,                           /// q0 or q1 (pace)
                  delta,                            /// Number of steps since last RM state change
                  lastAction_,                      /// Last action
		              lastlastAction_,                  /// Last last action
                  vel_command_x,                    /// Desired velocity along x
                  vel_command_y,                    /// Desired velocity along y
                  yaw;                              /// Desired yaw
  }


  void reset() final {
    // CHANGE DESIRED VELOCITY USING A LOOP
    vd_change = vd_change + 1;
    if (vd_change < 5)
      vel_command_x = 0;
      vel_command_y = 0;
    if (vd_change >= 5 && vd_change < 10)
      vel_command_x = 0.5;
      vel_command_y = 0;
    if (vd_change >= 10 && vd_change < 15) {
      vel_command_x = 1;
      vel_command_y = 0;
    }
    if (vd_change >= 15 && vd_change < 20) {
      vel_command_x = 0;
      vel_command_y = 0.5;
    }
    if (vd_change >= 20 && vd_change < 25) {
      vel_command_x = 0;
      vel_command_y = 1;
    }
    if (vd_change >= 25 && vd_change < 30) {
      vel_command_x = 0.5;
      vel_command_y = 0.5;
    }
    if (vd_change >= 30 && vd_change < 35) {
      vel_command_x = 1;
      vel_command_y = 1;
    }
    if (vd_change >= 35) {
      vel_command_x = 0;
      vel_command_y = 0;
      vd_change = 0;
    }
    
    // Reset all variables to zero
    anymal_->setState(gc_init_, gv_init_);
    updateObservation();
    P_1 = 0; P_2 = 0; P_3 = 0; P_4 = 0;
    q_trot = 0; q_bound = 0; q_pace = 0; delta = 0;
    lastAction_.setZero();
    lastlastAction_.setZero();
    yaw = 0;
  }


  float step(const Eigen::Ref<EigenVec>& action) final {
    /// action scaling
    pTarget12_ = action.cast<double>();
    pTarget12_ = pTarget12_.cwiseProduct(actionStd_);
    pTarget12_ += actionMean_;
    pTarget_.tail(nJoints_) = pTarget12_;

    anymal_->setPdTarget(pTarget_, vTarget_);

    for(int i=0; i< int(control_dt_ / simulation_dt_ + 1e-10); i++) {
      if(server_) server_->lockVisualizationServerMutex();
      world_->integrate();
      if(server_) server_->unlockVisualizationServerMutex();
    }

    updateObservation();

    // Torque penalty
    double torque = anymal_->getGeneralizedForce().squaredNorm();

    // Action smoothness penalty
    double ActionSmoothness1 = (pTarget12_ - lastAction_).squaredNorm();
    double ActionSmoothness2 = (pTarget12_ - 2*lastAction_ + lastlastAction_).squaredNorm();

    // JointPos penalty
    double jointsPos = (gc_.tail(nJoints_) - gc_init_.tail(nJoints_)).squaredNorm();
    double jointsVel = gv_.tail(nJoints_).squaredNorm();

    // Z vel penalty
    double bodyVel = std::pow(bodyLinearVel_[2],2);

    // Foot slip penalty
    double footSlip = 0.0;
    double cidx = 0;
    raisim::Vec<3> cvel, cpos;
    for(auto& contact: anymal_->getContacts()){
        if(footIndices_.find(contact.getlocalBodyIndex()) != footIndices_.end()){
            anymal_->getContactPointVel(cidx, cvel);
            footSlip += cvel.e().head(2).squaredNorm();
	      }
	    cidx += 1;
    }

    // Penalty to increase feet height when they are swing feet
    double footClearance = 0.0;
    anymal_->getFramePosition(anymal_->getFrameByName("FL_foot_fixed"), cpos);
    anymal_->getFrameVelocity(anymal_->getFrameByName("FL_foot_fixed"), cvel);
    footClearance += std::pow(cpos[2] - 0.1, 2) * std::pow(cvel.e().head(2).norm(),0.5);
    anymal_->getFramePosition(anymal_->getFrameByName("RL_foot_fixed"), cpos);
    anymal_->getFrameVelocity(anymal_->getFrameByName("RL_foot_fixed"), cvel);
    footClearance += std::pow(cpos[2] - 0.1, 2) * std::pow(cvel.e().head(2).norm(),0.5);
    anymal_->getFramePosition(anymal_->getFrameByName("FR_foot_fixed"), cpos);
    anymal_->getFrameVelocity(anymal_->getFrameByName("FR_foot_fixed"), cvel);
    footClearance += std::pow(cpos[2] - 0.1, 2) * std::pow(cvel.e().head(2).norm(),0.5);
    anymal_->getFramePosition(anymal_->getFrameByName("RR_foot_fixed"), cpos);
    anymal_->getFrameVelocity(anymal_->getFrameByName("RR_foot_fixed"), cvel);
    footClearance += std::pow(cpos[2] - 0.1, 2) * std::pow(cvel.e().head(2).norm(),0.5);
    
    // Negative reward
    double reward_neg = -0.05*jointsPos - 10*bodyVel - 0.5*jointsVel
                        -100*std::abs(gc_[7] - 0.06) -100*std::abs(gc_[10] + 0.06) - 100*std::abs(gc_[13] - 0.06)- 100*std::abs(gc_[16] + 0.06)
                        -1000*std::pow(bodyAngularVel_[2],2) - 0.1*std::pow(bodyAngularVel_[0],2) - 0.1*std::pow(bodyAngularVel_[1],2)
                        -10*footSlip - 100*footClearance - 40*ActionSmoothness1 - 40*ActionSmoothness2
                        -5000*std::abs(yaw);
                      
    // Positive reward 
    double reward_pos = 10.*std::exp(-0.8*std::pow(bodyLinearVel_[0] - vel_command_x,2))
                       +10.*std::exp(-0.8*std::pow(bodyLinearVel_[1] - vel_command_y,2)) - 4e-3*torque;

    // GET THE VECTOR P = {P_1, P_2, P_3, P_4} FOR REWARD MACHINES
    P_1 = 0;
    P_2 = 0;
    P_3 = 0;
    P_4 = 0;
    for(auto& contact: anymal_->getContacts()) {
      if(foot1_index == contact.getlocalBodyIndex())
        P_1 = 1;
      if(foot2_index == contact.getlocalBodyIndex())
        P_2 = 1;
      if(foot3_index == contact.getlocalBodyIndex())
        P_3 = 1;
      if(foot4_index == contact.getlocalBodyIndex())
        P_4 = 1;
    }

    // REWARD MACHINES
    if (delta > 0)
      delta -= 1;
    
    // Trot gait
    if(delta == 0 && P_1 == 1 && P_2 == 0 && P_3 == 0 && P_4 == 1 && q_trot == 0) {
      q_trot = 1;
      reward_pos = reward_pos + 400;
      delta = 10; 
    }
    if(delta == 0 && P_1 == 0 && P_2 == 1 && P_3 == 1 && P_4 == 0 && q_trot == 1)  {
      q_trot = 0;
      reward_pos = reward_pos + 400;
      delta = 10;
    }

    // Bound gait
    if(delta == 0 && P_1 == 1 && P_2 == 1 && P_3 == 0 && P_4 == 0 && q_bound == 0) {
      q_bound = 1;
      //reward_pos = reward_pos + 400;
      delta = 10; 
    }
    if(delta == 0 && P_1 == 0 && P_2 == 0 && P_3 == 1 && P_4 == 1 && q_bound == 1)  {
      q_bound = 0;
      //reward_pos = reward_pos + 400;
      delta = 10;
    }
    
    // Pace gait
    if(delta == 0 && P_1 == 1 && P_2 == 0 && P_3 == 1 && P_4 == 0 && q_pace == 0) {
      q_pace = 1;
      //reward_pos = reward_pos + 400;
      delta = 10; 
    }
    if(delta == 0 && P_1 == 0 && P_2 == 1 && P_3 == 0 && P_4 == 1 && q_pace == 1)  {
      q_pace = 0;
      //reward_pos = reward_pos + 400;
      delta = 10;
    }
    
    // Total reward
    double reward_total = reward_pos + 0.01*reward_neg;
    
    rewards_.record("reward_total", reward_total);

    lastlastAction_ = lastAction_;
    lastAction_ = pTarget12_;

    return rewards_.sum();
  }


  void observe(Eigen::Ref<EigenVec> ob) final {
    /// convert it to float
    ob = obDouble_.cast<float>();
  }


  bool isTerminalState(float& terminalReward) final {
    terminalReward = float(terminalRewardCoeff_);

    //if the contact body is not feet
    for(auto& contact: anymal_->getContacts())
      if(footIndices_.find(contact.getlocalBodyIndex()) == footIndices_.end())
        return true;

    terminalReward = 0.f;
    return false;
  }


  void curriculumUpdate() { 
    // Curriculum learning not used
    //if (kc < 1.)
      //kc += 0.0003;
  };


 private:
  int gcDim_, gvDim_, nJoints_, vd_change = 0, delta = 0, foot1_index = 0, foot2_index = 0, foot3_index = 0, foot4_index = 0, P_1 = 0, P_2 = 0, P_3 = 0, P_4 = 0, q_trot = 0, q_bound = 0, q_pace = 0;
  double vel_command_x = 0, vel_command_y = 0, yaw = 0;
  bool visualizable_ = false;
  double terminalRewardCoeff_ = -200.;
  Eigen::VectorXd gc_init_, gv_init_, gc_, gv_, pTarget_, pTarget12_, vTarget_;
  Eigen::VectorXd actionMean_, actionStd_, obDouble_, contatti;
  Eigen::VectorXd lastAction_, lastlastAction_;
  Eigen::Vector3d bodyLinearVel_, bodyAngularVel_,g;
  std::set<size_t> footIndices_;
  raisim::ArticulatedSystem* anymal_;
  raisim::HeightMap *hm;
  float kc = 0.1;


  /// these variables are not in use. They are placed to show you how to create a random number sampler.
  std::normal_distribution<double> normDist_;
  thread_local static std::mt19937 gen_;
};


thread_local std::mt19937 raisim::ENVIRONMENT::gen_;


}
