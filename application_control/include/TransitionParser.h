/*
 * TransitionPaser.h
 *
 *  Created on: Mar 22, 2012
 *      Author: Peter Hohnloser
 */

#ifndef TRANSITIONPARSER_H_
#define TRANSITIONPARSER_H_

#include "yaml-cpp/yaml.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <ros/ros.h>

#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

using std::map;
using std::string;
using std::vector;
using namespace YAML;

/**
 * Class TransitionParser parses a yaml file into the transitions between states
 */
class TransitionParser
{
public:
  /*
   * Constructor
   */
  TransitionParser()
  {
  }
  ;
  /*
   * parses a yaml file into a map with keys of strings and mapped values as a(nother) map of string keys and a string as the mapped value
   */
  map<string, map<string, string> > parser(string file_name, map<string, string> * state_description,
                                           string *strat_state);
};


map<string, map<string, string> > TransitionParser::parser(string file_name, map<string, string> * state_description,
                                                           string *start_state)
{
  string path(file_name.substr(0, file_name.size() - 4));
  path += "dot";
  ROS_INFO("transition file path: %s", file_name.c_str());
  ROS_INFO("dot file path: %s ", path.c_str());
  std::ofstream dotfile_(path.c_str());
  dotfile_ << "digraph{\n";

  map<std::string, map<std::string, std::string> > transitions_;
  try
  {
    ROS_DEBUG("File_name: %s", file_name.c_str());
    Node doc = LoadFile(file_name);
    ROS_DEBUG("YAML FILE LODED ");
    (*start_state) = doc[0]["start_state"].as<string>().c_str();
    for (int i = 1; i < doc.size(); i++)
    {
      (*state_description)[doc[i]["state"]["state_label"].as<string>()] =
          doc[i]["state"]["descriptive_name"].as<string>();
      ROS_DEBUG("%s", doc[i]["state"]["descriptive_name"].as<string>().c_str());
      ROS_DEBUG("%s", doc[i]["state"]["state_label"].as<string>().c_str());
      Node trans = doc[i]["state"]["transitions"];
      for (const_iterator it = trans.begin(); it != trans.end(); ++it)
      {
        transitions_[doc[i]["state"]["state_label"].as<string>()][it->first.as<string>()] = it->second.as<string>();
        ROS_DEBUG("%s : %s", it->first.as<string>().c_str(), it->second.as<string>().c_str());
        dotfile_ << doc[i]["state"]["state_label"].as<string>() << "->" << it->second.as<string>() << " [ label=" << '"'
            << it->first << '"' << " ];" << "\n";
      }
    }
  }
  catch (ParserException& e)
  {
    ROS_ERROR("Error parsing the yaml-file. Do not use tabs, only spaces");
    ros::shutdown();
  }
  dotfile_ << "}\n";
  dotfile_.close();
  ROS_INFO("%s file created", path.c_str());
  string execute("dot -Tps ");
  execute += path;
  execute += " -o ";
  execute += path.substr(0, path.size() - 4);
  execute += ".pdf";
  if (0 != system(execute.c_str()))
  {
    ROS_ERROR("Couldn't create a pdf from %s.", path.c_str());
  }

  return transitions_;
}

#endif /* TRANSITIONPASER_H_ */
