#ifndef PUBLISHCLOCK_H
#define PUBLISHCLOCK_H

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <thread>

class PublishClock
{
public:
  PublishClock(std::unique_ptr<ros::Publisher> publisher, double stamp_of_first_data, float playback_rate  = 1., unsigned int rate = 100);
  ~PublishClock();

  void start_publishing();

private:
  void timer_callback();
  void timer_thread();

  boost::asio::io_service io_service_;
  std::shared_ptr<boost::asio::deadline_timer> timer_;
  std::shared_ptr<std::thread> runner_;

  std::unique_ptr<ros::Publisher> publisher_;
  rosgraph_msgs::Clock ros_time_;
  float playback_rate_;
  unsigned int rate_;
};

#endif // PUBLISHCLOCK_H
