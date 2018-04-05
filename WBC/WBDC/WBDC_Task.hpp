#ifndef WHOLE_BODY_CONTROL_TASK
#define WHOLE_BODY_CONTROL_TASK

#include <Task.hpp>

class WBDC_Task: public Task{
public:
  WBDC_Task(int dim);
  virtual ~WBDC_Task();

  int getRelaxed_Dim(){ return dim_relaxed_;}
  void setRelaxedOpCtrl(const std::vector<bool> & relaxed_op);
  void getSdelta(dynacore::Matrix & Sd){ Sd = S_del_; }
protected:
  virtual bool _AdditionalUpdate(){ return true;}
  int dim_relaxed_;
  dynacore::Matrix S_del_;
};

#endif
