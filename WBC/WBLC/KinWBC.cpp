#include "KinWBC.hpp"
#include <Utils/utilities.hpp>
#include <Utils/pseudo_inverse.hpp>

KinWBC::KinWBC(int num_qdot, int num_act_joint, 
        const std::vector<int> & act_joint_idx):
    num_qdot_(num_qdot), num_act_joint_(num_act_joint),
    threshold_(0.00001)
{
    act_jidx_ = act_joint_idx;
    I_mtx = dynacore::Matrix::Identity(num_qdot, num_qdot);
}

bool KinWBC::FindConfiguration(
        const dynacore::Vector & curr_config,
        const std::vector<KinTask*> & task_list,
        const std::vector<ContactSpec*> & contact_list,
        dynacore::Vector & jpos_cmd,
        dynacore::Vector & jvel_cmd,
        dynacore::Vector & jacc_cmd){

    // Contact Jacobian Setup
    dynacore::Matrix Jc, Jc_i;
    contact_list[0]->getContactJacobian(Jc);
    int num_rows = Jc.rows();
    for(int i(1); i<contact_list.size(); ++i){
        contact_list[i]->getContactJacobian(Jc_i);
        int num_new_rows = Jc_i.rows();
        Jc.conservativeResize(num_rows, num_qdot_);
        Jc.block(num_rows, 0, num_new_rows, num_qdot_) = Jc_i;
        num_rows += num_new_rows;
    }

    // Projection Matrix
    dynacore::Matrix Nc;
    _BuildProjectionMatrix(Jc, Nc);

    dynacore::Vector delta_q, qdot, qddot, JtDotQdot;
    dynacore::Matrix Jt, JtPre, JtPre_pinv, N_pre;
    
    // First Task
    KinTask* task = task_list[0];
    task->getTaskJacobian(Jt);
    task->getTaskJacobianDotQdot(JtDotQdot);
    JtPre = Jt * Nc;
    dynacore::pseudoInverse(JtPre, threshold_, JtPre_pinv);
    delta_q = JtPre_pinv * (task->pos_err_);
    qdot = JtPre_pinv * (task->vel_des_);
    qddot = JtPre_pinv * (task->acc_des_ - JtDotQdot);

    dynacore::Vector prev_delta_q = delta_q;
    dynacore::Vector prev_qdot = qdot;
    dynacore::Vector prev_qddot = qddot;

    _BuildProjectionMatrix(JtPre, N_pre);

    for (int i(1); i<task_list.size(); ++i){
        task = task_list[i];

        task->getTaskJacobian(Jt);
        task->getTaskJacobianDotQdot(JtDotQdot);
        JtPre = Jt * N_pre;

        dynacore::pseudoInverse(JtPre, threshold_, JtPre_pinv);
        delta_q = prev_delta_q + JtPre_pinv * (task->pos_err_ - Jt * prev_delta_q);
        qdot = prev_qdot + JtPre_pinv * (task->vel_des_ - Jt* prev_qdot);
        qddot = prev_qddot + JtPre_pinv * (task->acc_des_ - JtDotQdot - Jt*prev_qddot);

        // For the next task
        _BuildProjectionMatrix(JtPre, N_pre);
        prev_delta_q = delta_q;
        prev_qdot = qdot;
        prev_qddot = qddot;
    }
    
    for(int i(0); i<num_act_joint_; ++i){
        jpos_cmd[i] = curr_config[act_jidx_[i]] + delta_q[act_jidx_[i]];
        jvel_cmd[i] = qdot[act_jidx_[i]];
        jacc_cmd[i] = qddot[act_jidx_[i]];
    }

    return true;
}

void KinWBC::_BuildProjectionMatrix(const dynacore::Matrix & J, 
                                    dynacore::Matrix & N){
    dynacore::Matrix J_pinv;
    dynacore::pseudoInverse(J, 0.00001, J_pinv);
    N = I_mtx  - J_pinv * J;
 }
