

#ifndef UR_DO_OUTPUT_H_
#define UR_DO_OUTPUT_H_

#ifdef ROS_BUILD
#include <ros/ros.h>
#endif
#include <string>

void print_debug(std::string inp);
void print_info(std::string inp);
void print_warning(std::string inp);
void print_error(std::string inp);
void print_fatal(std::string inp);


#endif /* UR_DO_OUTPUT_H_ */
