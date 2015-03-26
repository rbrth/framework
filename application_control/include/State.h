/*
 * State.h
 *
 *  Created on: Mar 1, 2012
 *      Author: Peter Hohnloser
 */
#ifndef STATE_H
#define STATE_H

#include <ros/ros.h>
#include "string.h"
#include <boost/thread/mutex.hpp>
#include <boost/any.hpp>


using std::string;
using std::map;
using boost::any;
using boost::mutex;
/*
 *  The class state is an abstract class which means
 *  that another class has to inherit from it.
 */
class State
{
private:
  string state_name_;
//  ros::NodeHandle nh_;

protected:
  bool pause;
  mutex mx;
  ros::Publisher stat_pub_;
  ros::NodeHandle nh_;

public:

  /*
   *Constructs a state which is used by the state machine
   */
  State()
  {
    pause = false;
    mutex::scoped_lock mylock(mx, boost::defer_lock);
  }
  ;

  virtual ~State()
  {
  }
  ;

  /**
   * -------------------- These methods are used in the state machine-------
   */

  /*
   * Sets the name for the state.
   */
  void setStateName(string state_name)
  {
    state_name_ = state_name;
  }
  ;
  /*
   * returns the name of the state
   */
  string getStateName()
  {
    return state_name_;
  }
  ;

  /*
   * This method is needed in the state machine for executing a state.
   * This method must be implemented in the class which inherits from State.
   */
  virtual string execute(map<string, any> * statedata)=0;

  /*
   * This method sets the state to pause.
   * It is possible to override this method for customize what happens when the state gets a pause request.
   */
  virtual bool setPause(bool p)
  {
    mx.lock();
    pause = p;
    mx.unlock();
    return true;
  }
  /**
   * This method returns true if the state is paused, otherwise false
   */
  bool getPause()
  {
    mx.lock();
    bool temp = pause;
    mx.unlock();
    return temp;
  }

  virtual void stop(){};

};

#endif
