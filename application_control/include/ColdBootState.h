/*
 * ColdBoot.h
 */

#ifndef COLDBOOT_H_
#define COLDBOOT_H_

#include "State.h"
#include "ros/ros.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "std_msgs/Bool.h"
#include "baxter_core_msgs/AssemblyState.h"

class ColdBootState : public State
{
private:


public:

    ros::NodeHandle   node_handle;
    ros::Publisher    robot_activation;
    ros::Subscriber   robot_state_subscriber;
    std::string       transition_message;
    bool              robot_state_enabled;
  
    void robotStateCallback(const baxter_core_msgs::AssemblyState::ConstPtr& state_msg);
    ColdBootState();
    std::string execute(std::map<std::string, boost::any> * data);
};

#endif /* COLDBOOT_H_ */
