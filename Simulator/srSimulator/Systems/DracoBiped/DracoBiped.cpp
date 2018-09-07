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
  collision_.resize(4);
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
  vp_joint_[2]->m_State.m_rValue[0] = 1.084217;// + 0.3;
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
