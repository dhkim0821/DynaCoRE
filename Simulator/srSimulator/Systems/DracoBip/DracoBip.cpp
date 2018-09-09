#include "DracoBip.h"

DracoBip::DracoBip():SystemGenerator(), hanging_height_( 1.084217  + 0.3 )
{
  printf("[DracoBip] ASSEMBLED\n");
}

DracoBip::~DracoBip(){
}

void DracoBip::_SetJointLimit(){
}

void DracoBip::_SetCollision(){
  collision_.resize(4 + 1);
  for (int i = 0; i < collision_.size(); ++i) {
    collision_[i] = new srCollision();
  }

  collision_[0]->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
  collision_[0]->GetGeomInfo().SetDimension(0.055, 0.0, 0.0);
  collision_[0]->SetLocalFrame(EulerZYX(Vec3(0,0,0), Vec3(0.07, 0, 0)));
 
  collision_[1]->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
  collision_[1]->GetGeomInfo().SetDimension(0.055, 0.0, 0.0);
  collision_[1]->SetLocalFrame(EulerZYX(Vec3(0,0,0), Vec3(-0.06, 0, 0)));
  
  collision_[2]->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
  collision_[2]->GetGeomInfo().SetDimension(0.055, 0.0, 0.0);
  collision_[2]->SetLocalFrame(EulerZYX(Vec3(0,0,0), Vec3(0.07, 0.0, 0)));

  collision_[3]->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
  collision_[3]->GetGeomInfo().SetDimension(0.055, 0.0, 0.0);
  collision_[3]->SetLocalFrame(EulerZYX(Vec3(0,0,0), Vec3(-0.06, 0., 0)));


  link_[link_idx_map_.find("rAnkle")->second]->AddCollision(collision_[0]);
  link_[link_idx_map_.find("rAnkle")->second]->AddCollision(collision_[1]);
  link_[link_idx_map_.find("lAnkle")->second]->AddCollision(collision_[2]);
  link_[link_idx_map_.find("lAnkle")->second]->AddCollision(collision_[3]);

  double fric(0.8);
  link_[link_idx_map_.find("rAnkle")->second]->SetFriction(fric);
  link_[link_idx_map_.find("lAnkle")->second]->SetFriction(fric);

  double damp(0.01);
  link_[link_idx_map_.find("rAnkle")->second]->SetDamping(damp);
  link_[link_idx_map_.find("lAnkle")->second]->SetDamping(damp);

  double restit(0.0);
  link_[link_idx_map_.find("rAnkle")->second]->SetRestitution(restit);
  link_[link_idx_map_.find("lAnkle")->second]->SetRestitution(restit);

  // Link
  collision_[4] = new srCollision();
  collision_[4]->GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
  collision_[4]->GetGeomInfo().SetDimension(0.05, 0.1, 0.05);
  collision_[4]->SetLocalFrame(EulerZYX(Vec3(0.,0., 0.), Vec3(0., 0., -hanging_height_ + 0.050)));
  link_[link_idx_map_.find("torso")->second]->AddCollision(collision_[4]);
  link_[link_idx_map_.find("torso")->second]->SetFriction(fric);

}

void DracoBip::_SetInitialConf(){

    for(int i(0); i<3; ++i)    vr_joint_[i]->m_State.m_rValue[1] = 0.;
    for(int i(0); i<3; ++i)    vp_joint_[i]->m_State.m_rValue[1] = 0.;
    for(int i(0); i<num_act_joint_; ++i)    r_joint_[i]->m_State.m_rValue[1] = 0.;

  vp_joint_[0]->m_State.m_rValue[0] = 0.0;
  vp_joint_[1]->m_State.m_rValue[0] = 0.0;
  //vp_joint_[2]->m_State.m_rValue[0] = 1.084217;
  vp_joint_[2]->m_State.m_rValue[0] = hanging_height_;
  vr_joint_[0]->m_State.m_rValue[0] = 0.0;
  vr_joint_[1]->m_State.m_rValue[0] = 0.0348;
  vr_joint_[2]->m_State.m_rValue[0] = 0.0;

  r_joint_[r_joint_idx_map_.find("lHipPitch")->second]->m_State.m_rValue[0] = -0.59;
  r_joint_[r_joint_idx_map_.find("lKnee")->second]->m_State.m_rValue[0] = 1.1;
  r_joint_[r_joint_idx_map_.find("lAnkle")->second]->m_State.m_rValue[0] = 1.03;

  r_joint_[r_joint_idx_map_.find("rHipPitch")->second]->m_State.m_rValue[0] = -0.59;
  r_joint_[r_joint_idx_map_.find("rKnee")->second]->m_State.m_rValue[0] = 1.1;
  r_joint_[r_joint_idx_map_.find("rAnkle")->second]->m_State.m_rValue[0] = 1.03;

  KIN_UpdateFrame_All_The_Entity();
}
