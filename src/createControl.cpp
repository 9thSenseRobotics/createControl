// This controls a robot base with input commands from skype chat
// It is derived from TurtlebotTeleop in the Turtlebot package from Willow Garage
// Copyright (c) 2011, 9th Sense, Inc.
// All rights reserved.

// For the Willow Garage parts of the code:
/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 */
 
 // For all of this code:
 
 /*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

 
#define DRIVING_NONE 0
#define DRIVING_FORWARD 1
#define DRIVING_BACKWARD 2
#define DRIVING_TURNLEFT 3
#define DRIVING_TURNRIGHT 4

#define MAX_FORWARD_SPEED .500  // speeds are in m/sec
#define MAX_BACKWARD_SPEED -.500
#define MAX_TURN_SPEED 10		// turns are in radians/sec
#define MIN_FORWARD_SPEED .050
#define MIN_BACKWARD_SPEED -.050
#define MIN_TURN_SPEED 1.
#define DELTA_FORWARD_SPEED .050
#define DELTA_BACKWARD_SPEED -.050
#define DELTA_TURN_SPEED 1.
#define DEFAULT_FORWARD_SPEED .300
#define DEFAULT_BACKWARD_SPEED -.300
#define DEFAULT_TURN_SPEED 3.

#define ARDUINO_FILENAME "/dev/ttyACM0"

#define RAMPUP_SPEED_DELAY 200.  // controls how long between speed steps when ramping from a stop to default speed
#define MIN_TURN_TIME 100.
#define MIN_MOVE_TIME 500.
#define TIME_OUT 8000.  // number of milliseconds to wait for command before you consider it a timeout and stop the base
#define TURN_OFF_CREATE 900000.  // number of milliseconds of inactivity before powering down create to save battery power (15 min = 900 seconds = 900000 msec)
#define TURTLEBOT_NODE_PREVENT_TIMEOUT 500	// for some reason, the timeout gets set as 1 second in turtlebot_node regardless of the input parameter, so we need to publish frequently to prevent the timeout

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
//#include "ros/console.h"
#include <std_msgs/String.h>

class TelebotSkypeCmd
{
public:
  TelebotSkypeCmd();

private:
  //void joyCallback(const joy::Joy::ConstPtr& joy);
  void skypeCallback(const std_msgs::String& msgSkype);
  void publish();

  ros::NodeHandle ph_, nh_;
  
  void stop();
  void slowStop();
  void moveForward();
  void moveForwardUntilToldToStop();
  void moveBackward();
  void turn();
  void togglePower();
  void delay(float delayMsec);

  int linear_, angular_, deadman_axis_;
  int numSteps, driving;
  double l_scale_, a_scale_;
  std_msgs::String ReceivedCommands;
  ros::Publisher vel_pub_, cmd_pub_;
  ros::Subscriber skypechat_sub;

  geometry_msgs::Twist vel, last_published_;
  boost::mutex publish_mutex_;
  bool deadman_pressed_;
  ros::Timer timer_;
};

TelebotSkypeCmd::TelebotSkypeCmd():
  ph_("~"),
  linear_(1),
  angular_(0),
  deadman_axis_(4),
  l_scale_(0.3),
  a_scale_(0.9)
{


  ph_.param("axis_linear", linear_, linear_);
  ph_.param("axis_angular", angular_, angular_);
  ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("turtlebot_node/cmd_vel", 1);
  cmd_pub_ = nh_.advertise<std_msgs::String>("robot_commands", 1);
  skypechat_sub = nh_.subscribe("SkypeChat", 10, &TelebotSkypeCmd::skypeCallback, this);
 // timer_ = nh_.createTimer(ros::Duration(TURTLEBOT_NODE_PREVENT_TIMEOUT/1000.), boost::bind(&TelebotSkypeCmd::publish, this));  // publish to keep the turtlebot node from timing out (it has a 1 second timeout right now)
  // but this does not go when we do ros.sleep, so when sleeping we have to have another method
// so we don't bother with this at all.
}

void TelebotSkypeCmd::skypeCallback( const std_msgs::String& msgSkype)
{ 
    ReceivedCommands = msgSkype;
    cmd_pub_.publish(ReceivedCommands);
    ROS_INFO("%s", ReceivedCommands.data.c_str());
    numSteps = strlen( (const char* ) ReceivedCommands.data.c_str());
    if ( numSteps > 10 ) numSteps = 2;  // might be mistake in held down key
    else if (numSteps > 5) numSteps = 5;  // if you just hit a few extra keys, go max
    for (int i = 1; i < numSteps; i++) if ( (msgSkype.data[i] != msgSkype.data[0]) && msgSkype.data[i] != msgSkype.data[0] + 32) return; 
          // if string is not all identical characters, allowing for first character to be a capital, return    
    char cmd = msgSkype.data[0]; 
    switch(cmd)
    {
      case 'z':    // stop
      case 'Z':    // stop
        ReceivedCommands.data = "command issued to slowly stop";
        cmd_pub_.publish(ReceivedCommands);
        slowStop();
        break;
        
      case 'x':    // stop
      case 'X':    // stop
        stop();
        ReceivedCommands.data = "command issued to stop";
        cmd_pub_.publish(ReceivedCommands);
        break;
      
      case 'w':  // move forward
      case 'W':  // move forward
        ReceivedCommands.data = "command issued to move forward";
        cmd_pub_.publish(ReceivedCommands);
        driving = DRIVING_FORWARD;
        moveForward();
        break;
	  case 'f':  // move forward
      case 'F':  // move forward
        ReceivedCommands.data = "command issued to move forward until told to stop";
        cmd_pub_.publish(ReceivedCommands);
        driving = DRIVING_FORWARD;
        moveForwardUntilToldToStop();
        break;        
      case 's':  //move backward
      case 'S':  //move backward
        ReceivedCommands.data = "command issued to move backward";
        cmd_pub_.publish(ReceivedCommands);
        driving = DRIVING_BACKWARD;
        moveBackward();
        break;
        
      case 'd':  //turn right
      case 'D':
        driving = DRIVING_TURNRIGHT;
        ReceivedCommands.data = "command issued to turn right";
        cmd_pub_.publish(ReceivedCommands);
        turn();
        break;
         
      case 'a':  // turn left
      case 'A':
        driving = DRIVING_TURNLEFT;
        ReceivedCommands.data = "command issued to turn left";
        cmd_pub_.publish(ReceivedCommands);
        turn();
        break;
      
      case 'p':    // toggle power
      case 'P':    // toggle power
        ReceivedCommands.data = "command issued to toggle Create power";
        cmd_pub_.publish(ReceivedCommands);
        togglePower();
        break;
        
      default:  // unknown command
        //stop();  w// we need to ignore, not stop because otherwise we will stop when people are just saying stuff like "hi"
        break;
    }
}

void TelebotSkypeCmd::togglePower()
{
	FILE * fp;	
	fp = fopen (ARDUINO_FILENAME, "w");
	fputc ('p', fp);
	fclose (fp);
}

void TelebotSkypeCmd::moveForward()
{
  vel.angular.z = 0;
  if (numSteps == 1)     // just get it done
  {      
  	vel.linear.x = DEFAULT_FORWARD_SPEED;
  	publish();
    delay(MIN_MOVE_TIME);
    slowStop();
  }
  else  // use ramp up and ramp down
  {
    vel.linear.x = MIN_FORWARD_SPEED;
    while (vel.linear.x <= DEFAULT_FORWARD_SPEED)  // ramp up to speed from a stop
    {
         publish();
         delay(RAMPUP_SPEED_DELAY);
         vel.linear.x += DELTA_FORWARD_SPEED;
    }
    for (int i = 1; i < numSteps; i++)
    {
      if (vel.linear.x + DELTA_FORWARD_SPEED <= MAX_FORWARD_SPEED)
      {
      	vel.linear.x += DELTA_FORWARD_SPEED;
        publish();
      }
      delay(RAMPUP_SPEED_DELAY);
    } 
    delay(MIN_MOVE_TIME * ( (3 * numSteps) - 6 ));  // numSteps here can range from 2 to 5, so we can get 0, 3MMT, 6MMT, and 9MMT    
    slowStop();
  }
}

void TelebotSkypeCmd::moveForwardUntilToldToStop()
{
  vel.angular.z = 0;    
  	vel.linear.x = DEFAULT_FORWARD_SPEED;
  	publish();
}


void TelebotSkypeCmd::moveBackward()
{
  vel.angular.z = 0;
  if (numSteps == 1) 
  {
    vel.linear.x = DEFAULT_BACKWARD_SPEED;
  	publish();
    delay(MIN_MOVE_TIME);
    slowStop();
  }
  else
  {
    vel.linear.x = MIN_BACKWARD_SPEED;
    while (vel.linear.x >= DEFAULT_BACKWARD_SPEED)  // ramp up to speed from a stop, note that these are negative numbers
    {
        publish();
        delay(RAMPUP_SPEED_DELAY);
        vel.linear.x += DELTA_BACKWARD_SPEED;
    }
    for (int i = 1; i < numSteps; i++)
    {
      if (vel.linear.x >= MAX_BACKWARD_SPEED) publish();
      delay(RAMPUP_SPEED_DELAY);
      vel.linear.x += DELTA_BACKWARD_SPEED;
    }
    delay(MIN_MOVE_TIME * ( (3 * numSteps) - 6 ));  // numSteps here can range from 2 to 5, so we can get 0, 3MMT, 6MMT, and 9MMT 
    if (numSteps > 3) delay(1000);  // for longer runs, add some extra time
    slowStop();
  }
}

void TelebotSkypeCmd::turn()
{
  vel.linear.x = 0;
  if (driving == DRIVING_TURNRIGHT) vel.angular.z = -DEFAULT_TURN_SPEED;
  else vel.angular.z = DEFAULT_TURN_SPEED;
  publish();
  delay(MIN_TURN_TIME * numSteps);
  if (numSteps > 3) delay(500);  // for longer runs, add some extra time
  if (numSteps == 1) stop();
  else slowStop();
 }


void TelebotSkypeCmd::slowStop()
{
  if (driving < 3) // not turning
  {
    vel.angular.z = 0;
    if (driving == DRIVING_FORWARD)
    {
		while (vel.linear.x > MIN_FORWARD_SPEED) 
		{
		    vel.linear.x -= DELTA_FORWARD_SPEED;
		    publish();
		    delay(RAMPUP_SPEED_DELAY);
		}
	}
	else
	{
		while (vel.linear.x < MIN_BACKWARD_SPEED)  // negative numbers here
		{
		    vel.linear.x -= DELTA_BACKWARD_SPEED;
		    publish();
		    delay(RAMPUP_SPEED_DELAY);
		}
    }

  }
  else  // turning
  {
    int absTurnSpeed = abs(vel.angular.z);
    vel.linear.x = 0;
    while (absTurnSpeed > MIN_TURN_SPEED)
    {
      absTurnSpeed -= DELTA_TURN_SPEED;
      if (driving == DRIVING_TURNRIGHT) vel.angular.z = -absTurnSpeed; 
      else vel.angular.z = absTurnSpeed; 
      publish();
      delay(RAMPUP_SPEED_DELAY);
    }
  }
  stop();
}

void TelebotSkypeCmd::stop()
{
  vel.angular.z = 0;
  vel.linear.x = 0;
  publish();
  last_published_ = vel;
  driving = DRIVING_NONE;
}

void TelebotSkypeCmd::delay(float delayMsec)
{
	while (delayMsec > TURTLEBOT_NODE_PREVENT_TIMEOUT)
	{
		 ros::Duration(TURTLEBOT_NODE_PREVENT_TIMEOUT/1000.).sleep(); 
		 delayMsec -= TURTLEBOT_NODE_PREVENT_TIMEOUT;
		 publish();
	}
	ros::Duration(delayMsec/1000.).sleep();
	publish(); 		
}

void TelebotSkypeCmd::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);
  vel_pub_.publish(vel);
  //ReceivedCommands.data = "velocities published";
  //ROS_INFO("%s", ReceivedCommands.data.c_str());
  //cmd_pub_.publish(ReceivedCommands);
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "createControl");
  TelebotSkypeCmd createControl;
  ros::spin();
}
