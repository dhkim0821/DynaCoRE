#include "WBDC_Task.hpp"
#include <Utils/utilities.hpp>

WBDC_Task::WBDC_Task(int dim): Task(dim),
                               dim_relaxed_(0){
  // printf("[WBDC Task]Constructed\n");
}
WBDC_Task::~WBDC_Task(){}

void WBDC_Task::setRelaxedOpCtrl(const std::vector<bool> & relaxed_op){
  dim_relaxed_ = 0;
  for(int i(0); i<dim_task_; ++i){
    if(relaxed_op[i]){ ++dim_relaxed_; }
  }
  S_del_ = dynacore::Matrix::Zero(dim_task_, dim_relaxed_);

  int col(0);
  for(int i(0); i<dim_task_; ++i){
    if(relaxed_op[i]){
      S_del_(i, col) = 1.;
      ++col;
    }
  }
  // dynacore::pretty_print(S_del_, std::cout, "S delta");
}
