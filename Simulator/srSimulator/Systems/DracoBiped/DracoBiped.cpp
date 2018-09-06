#include "DracoBiped.h"

DracoBiped::DracoBiped():SystemGenerator()
{
  printf("[DracoBiped] ASSEMBLED\n");
}

DracoBiped::~DracoBiped(){
}

void DracoBiped::_SetJointLimit(){
}

void DracoBiped::_SetCollision(){
  collision_.resize(2);
  for (int i = 0; i < collision_.size(); ++i) {
    collision_[i] = new srCollision();
  }

  collision_[0]->GetGeomInfo().SetShape(srGeometryInfo::BOX);
  collision_[0]->GetGeomInfo().SetDimension(0.23, 0.16, 0.03);
  collision_[1]->GetGeomInfo().SetShape(srGeometryInfo::BOX);
  collision_[1]->GetGeomInfo().SetDimension(0.23, 0.16, 0.03);

  //link_[link_idx_map_.find("r_foot")->second]->AddCollision(collision_[0]);
  //link_[link_idx_map_.find("l_foot")->second]->AddCollision(collision_[1]);

  //double fric(0.8);
  //link_[link_idx_map_.find("r_foot")->second]->SetFriction(fric);
  //link_[link_idx_map_.find("l_foot")->second]->SetFriction(fric);

  //double damp(0.01);
  //link_[link_idx_map_.find("r_foot")->second]->SetDamping(damp);
  //link_[link_idx_map_.find("l_foot")->second]->SetDamping(damp);

  //double restit(0.0);
  //link_[link_idx_map_.find("r_foot")->second]->SetRestitution(restit);
  //link_[link_idx_map_.find("l_foot")->second]->SetRestitution(restit);
}

void DracoBiped::_SetInitialConf(){
  vp_joint_[0]->m_State.m_rValue[0] = 0.0;
  vp_joint_[1]->m_State.m_rValue[0] = 0.0;
  vp_joint_[2]->m_State.m_rValue[0] = 1.135;// + 0.3;
  vr_joint_[0]->m_State.m_rValue[0] = 0.0;
  vr_joint_[1]->m_State.m_rValue[0] = 0.0;
  vr_joint_[2]->m_State.m_rValue[0] = 0.0;

  KIN_UpdateFrame_All_The_Entity();
}
