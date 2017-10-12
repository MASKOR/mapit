#ifndef PUBLISHCLOCK_H
#define PUBLISHCLOCK_H

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

class PublishClock
{
public:
  PublishClock(std::unique_ptr<ros::Publisher> publisher, float playback_rate  = 1., unsigned int rate = 100);

  //TODO callback that publishes the clock

private:
  std::unique_ptr<ros::Publisher> publisher_;
  rosgraph_msgs::Clock clock;
  float playback_rate_;
  unsigned int rate_;
};

#endif // PUBLISHCLOCK_H
