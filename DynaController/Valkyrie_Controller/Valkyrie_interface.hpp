#ifndef VALKYRIE_INTERFACE_H
#define VALKYRIE_INTERFACE_H

#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>
#include <interface.hpp>
#include <Valkyrie/Valkyrie_Definition.h>

class Valkyrie_StateProvider;

class Valkyrie_SensorData{
    public:
        double q[valkyrie::num_q];
        double qdot[valkyrie::num_qdot];
};

class Valkyrie_interface : public interface{
    public:
        Valkyrie_interface();
        virtual ~Valkyrie_interface();

        virtual void GetCommand(void* sensor_data, std::vector<double> & command);

    protected:
        Valkyrie_StateProvider* sp_;
        bool _Initialization(void * sensor_data);
        dynacore::Vector initial_jpos_;
        dynacore::Vector gamma_;
};

#endif
