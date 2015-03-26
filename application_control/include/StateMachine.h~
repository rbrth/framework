/*
 * StateMachine.h
 *
 *  Created on: Mar 1, 2012
 *      Author: Peter Hohnloser
 */
#ifndef STATEMACHINE_H
#define STATEMACHINE_H
#include <map>
#include "State.h"
#include <ros/ros.h>
#include <unistd.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/any.hpp>
#include <std_msgs/String.h>
#include <rosgraph_msgs/Log.h>

#include <TransitionParser.h>
#include <application_control/changeState.h>
#include <application_control/State.h>
#include <application_control/PauseRequest.h>

using std::map;
using std::string;
using boost::thread;
using boost::mutex;

class StateMachine
{
private:
  volatile bool run_;
  ros::NodeHandle n_;
  ros::Publisher curr_state_pub_;
  map<string, map<string, string> > transitions_;
  map<string, string> state_description_;
  map<string, State *> states_;
  map<string, any> state_data_;
  State * current_state_;
  string *out_;
  string next_state_;
  string start_state_;
  int outSize_;
  thread rosspin_;
  double frequency_;
  double pause_start_time_;
  mutex next;
  // Diagnostics:
  double min_freq_, max_freq_;
public:

  /**
   * Constructor for a State Machine
   *
   */
  StateMachine(string outcomes[], int outcomeSize);

  virtual ~StateMachine()
  {
  }
  ;
  /**
   *  Adds a State to the State Machine
   */
  void add(string label, State *state);

  //  void add(string label, State *state,string transition[], int n);
  /*
   *  adds the yaml file to the state machine and creates the transition between the states
   */
  void addTransitionFile(string file_name);
  /**
   * executes the State Machine
   */
  string execute();

private:
  /**
   * Checks the transitions between states
   */
  void checkTransitions();
  /*
   * initialize ros and creates a pause request service thread
   *
   */
  void init();
  /*
   *  method which a pause request service thread carries out
   */
  void rosSpin();
  /*
   * Callback method for the pause request service which sets a states to pause
   */
  bool pause(application_control::PauseRequest::Request &rep, application_control::PauseRequest::Response &res);

  /**
   * Callback methed for changing a state
   */

  void changeState(const application_control::changeState::ConstPtr& msg);

};


StateMachine::StateMachine(string outcomes[], int outcomeSize)
{
  out_ = outcomes;
  outSize_ = outcomeSize;
  init();
}

void StateMachine::add(string label, State *state)
{
  state->setStateName(label);
  states_[label] = state;
}

string StateMachine::execute()
{
  string last_state_, trans_;
  application_control::State state_msg;
  int i = 0;
  bool hasFinishd_ = false;
  checkTransitions();
  ros::Rate loop_rate(100);
  current_state_ = states_[start_state_];
  double start_time = ros::Time::now().toSec();
  while (ros::ok())
  {

    if (!current_state_->getPause())
    {
      state_msg.description = state_description_[current_state_->getStateName()];
      state_msg.name = current_state_->getStateName();
      state_msg.header.stamp = ros::Time::now();
      ROS_INFO("Current State: %s", state_msg.name.c_str());
      curr_state_pub_.publish(state_msg);
      last_state_ = current_state_->getStateName();
      trans_ = current_state_->execute(&state_data_);

      if (transitions_[last_state_].find(trans_) == transitions_[last_state_].end())
      {
        ROS_ERROR("#GUI State %s has no outcome %s ", last_state_.c_str(), trans_.c_str());
        run_ = false;
        rosspin_.join();
        exit(-1);
      }

      next_state_ = transitions_[last_state_][trans_];
      for (i = 0; i < outSize_; i++)
      {
        if (next_state_.compare(out_[i]) == 0)
        {
          hasFinishd_ = true;
          break;
        }
      }
      if (hasFinishd_)
      {
        break;
      }
      next.unlock();
      loop_rate.sleep();
      next.lock();
      if (current_state_->getStateName().compare(next_state_) != 0)
      {
        //ROS_INFO("#SUM %s Duration %f sec", current_state_->getStateName().c_str(),ros::Time::now().toSec() - start_time);
        start_time = ros::Time::now().toSec();
      }
      current_state_ = states_[next_state_];
    }
    else
    {
      loop_rate.sleep();
    }
  }
  state_msg.description = "State machine has finished";
  state_msg.name = "Exit";
  curr_state_pub_.publish(state_msg);
  run_ = false;
  rosspin_.join();
  ROS_INFO("State Machine exit");
  return out_[i];
}

void StateMachine::addTransitionFile(string file_name)
{
  TransitionParser tran;
  transitions_ = tran.parser(file_name, &state_description_, &start_state_);
}

/**
 *---------------------------------------------------------------Private methods--------------------------------------------------------------------
 */

void StateMachine::checkTransitions()
{
  map<string, map<string, string> >::iterator it;
  map<string, string>::iterator it2;
  int i;
  bool hasFailure_ = true;
  if (states_.find(start_state_) == states_.end())
  {
    ROS_ERROR("#GUI State machine has no start state %s", start_state_.c_str());
    ros::shutdown();
  }

  for (it = transitions_.begin(); it != transitions_.end(); it++)
  {
    map<string, string> &temp = it->second;
    for (it2 = temp.begin(); it2 != temp.end(); it2++)
    {

      if (states_.find(it2->second) == states_.end())
      {
        ROS_INFO("State Machine: state label [%s] outcome [%s] to [%s]", it->first.c_str(), it2->first.c_str(),
                  it2->second.c_str());
        for (i = 0; i < outSize_; i++)
        {
          if (out_[i].compare(it2->second) == 0)
          {
            hasFailure_ = false;
            break;
          }
        }
        if (hasFailure_)
        {
          ROS_ERROR("#GUI State ( %s )  outcome ( %s ) has no transition to next State ( %s )", it->first.c_str(),
                    it2->first.c_str(), it2->second.c_str());
          ros::shutdown();
        }
        hasFailure_ = true;
      }
    }
  }
}

void StateMachine::init()
{
  char *argv[1];
  int argc = 1;
  argv[0] = get_current_dir_name();
  run_ = true;
  ros::init(argc, argv, "StateMachine");
  curr_state_pub_ = n_.advertise<application_control::State>("current_state", 1);
  rosspin_ = thread(&StateMachine::rosSpin, this);
  ROS_INFO("State Machine initialized");
}

void StateMachine::rosSpin()
{
  ROS_INFO("State Machine I'm alive message started");
  ros::ServiceServer service_ = n_.advertiseService("pause_request", &StateMachine::pause, this);
  ros::Subscriber sub_ = n_.subscribe("change_state", 1, &StateMachine::changeState, this);
  ros::Publisher alive_pub_ = n_.advertise<std_msgs::String>("update", 1);
  ros::Rate loop_rate(100);
  std_msgs::String msg;
  msg.data = "I'm alive";
  while (run_)
  {
    ros::spinOnce();
    alive_pub_.publish(msg);
    ros::Time timestamp = ros::Time::now();
    loop_rate.sleep();
  }
}

bool StateMachine::pause(application_control::PauseRequest::Request &rep, application_control::PauseRequest::Response &res)
{
  ROS_INFO("State Machine pause requested");
  res.succeded = true;

  if (current_state_->setPause(rep.pause))
  {
    res.succeded = false;
  }

  if (rep.pause)
  {
    // start pause timer
    pause_start_time_ = ros::Time::now().toSec();
  }
  else
  {
    // end pause timer and log duration to file
    ROS_INFO("#SUM Pause duration %f sec, (state: %s)", ros::Time::now().toSec() - pause_start_time_,
             current_state_->getStateName().c_str());
  }
  return true;
}

void StateMachine::changeState(const application_control::changeState::ConstPtr& msg)
{
  ROS_INFO("State Machine change state to %s", msg->state_label.c_str());
  map<string, State*>::iterator it;
  current_state_->stop();
  for (it = states_.begin(); it != states_.end(); it++)
  {
    it->second->stop();
  }
  current_state_->setPause(false);
  next.lock();
  next_state_ = msg->state_label;
  next.unlock();

}

#endif // STATEMACHINE_H
