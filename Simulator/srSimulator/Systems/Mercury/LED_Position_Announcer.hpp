#ifndef LED_POS_ANNOUNCER
#define LED_POS_ANNOUNCER

#include "Utils/dynacore_pThread.hpp"

class Mercury_Dyn_environment;

class LED_Position_Announcer: public dynacore_pThread{
public:

    LED_Position_Announcer(Mercury_Dyn_environment* );
    virtual ~LED_Position_Announcer(void){}

    virtual void run(void);

protected:
    int socket_;
    Mercury_Dyn_environment* dyn_env_;
};

#endif
