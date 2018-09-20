#include "KinWBC.hpp"
#include <Utils/utilities.hpp>
#include <Utils/pseudo_inverse.hpp>

KinWBC::KinWBC(const std::vector<bool> & act_joint):
    num_act_joint_(0),
    threshold_(0.00001)
{
    num_qdot_ = act_joint.size();

    act_jidx_.clear(); 
    for(int i(0); i<num_qdot_; ++i){
        if(act_joint[i]){
            act_jidx_.push_back(i);
            ++num_act_joint_;
        }
    }
    //dynacore::pretty_print(act_jidx_, "act jidx");
    I_mtx = dynacore::Matrix::Identity(num_qdot_, num_qdot_);
}

bool KinWBC::FindConfiguration(
        const dynacore::Vector & curr_config,
        const std::vector<Task*> & task_list,
        const std::vector<ContactSpec*> & contact_list,
        dynacore::Vector & jpos_cmd,
        dynacore::Vector & jvel_cmd,
        dynacore::Vector & jacc_cmd){

    //printf("contact list size: %d\n", contact_list.size());
    // Contact Jacobian Setup
    dynacore::Matrix Jc, Jc_i;
    contact_list[0]->getContactJacobian(Jc);
    int num_rows = Jc.rows();
    for(int i(1); i<contact_list.size(); ++i){
        contact_list[i]->getContactJacobian(Jc_i);
        int num_new_rows = Jc_i.rows();
        Jc.conservativeResize(num_rows + num_new_rows, num_qdot_);
        Jc.block(num_rows, 0, num_new_rows, num_qdot_) = Jc_i;
        num_rows += num_new_rows;
    }

    // Projection Matrix
    dynacore::Matrix Nc;
    _BuildProjectionMatrix(Jc, Nc);

    dynacore::Vector delta_q, qdot, qddot, JtDotQdot;
    dynacore::Matrix Jt, JtPre, JtPre_pinv, N_pre;
    
    // First Task
    KinTask* task = ((KinTask*)task_list[0]);
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

    //dynacore::pretty_print(Jt, std::cout, "1st task Jt");
    //dynacore::pretty_print(Jc, std::cout, "Jc");
    //dynacore::pretty_print(Nc, std::cout, "Nc");
    //dynacore::pretty_print(JtPre, std::cout, "JtNc");
    //dynacore::pretty_print(JtPre_pinv, std::cout, "JtNc_inv");
    //dynacore::pretty_print(delta_q, std::cout, "delta q");

    for (int i(1); i<task_list.size(); ++i){
        task = ((KinTask*)task_list[i]);

        task->getTaskJacobian(Jt);
        task->getTaskJacobianDotQdot(JtDotQdot);
        JtPre = Jt * N_pre;

        dynacore::pseudoInverse(JtPre, threshold_, JtPre_pinv);
        delta_q = prev_delta_q + JtPre_pinv * (task->pos_err_ - Jt * prev_delta_q);
        qdot = prev_qdot + JtPre_pinv * (task->vel_des_ - Jt* prev_qdot);
        qddot = prev_qddot + JtPre_pinv * (task->acc_des_ - JtDotQdot - Jt*prev_qddot);

        //dynacore::pretty_print(Jt, std::cout, "2nd Jt");
        //dynacore::pretty_print(N_pre, std::cout, "N_pre");
        //dynacore::pretty_print(JtPre, std::cout, "JtPre");
        //dynacore::pretty_print(delta_q, std::cout, "delta q");

        // For the next task
        _BuildProjectionMatrix(JtPre, N_pre);
        prev_delta_q = delta_q;
        prev_qdot = qdot;
        prev_qddot = qddot;
    }
// TEST
    qdot.setZero();
    qddot.setZero();
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
    dynacore::pseudoInverse(J, 0.01, J_pinv);
    N = I_mtx  - J_pinv * J;
 }
