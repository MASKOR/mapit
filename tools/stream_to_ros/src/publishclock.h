/*******************************************************************************
 *
 * Copyright      2017 Tobias Neumann	<t.neumann@fh-aachen.de>
 *
******************************************************************************/

/*  This file is part of mapit.
 *
 *  Mapit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Mapit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
 */

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
