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

#include "publishclock.h"

#include <mapit/time/time.h>

PublishClock::PublishClock(std::unique_ptr<ros::Publisher> publisher, double stamp_of_first_data, float playback_rate, unsigned int rate)
  : publisher_( std::move(publisher) )
  , playback_rate_(playback_rate)
  , rate_(rate)
  , io_service_()
{
  ros_time_.clock = ros::Time( stamp_of_first_data );
}

PublishClock::~PublishClock()
{
  io_service_.stop();
  runner_->join();
}

void
PublishClock::start_publishing()
{
  // create the first boost timer
  timer_ = std::make_shared<boost::asio::deadline_timer>(io_service_, boost::posix_time::milliseconds(1000. / rate_));
  runner_ = std::make_shared<std::thread>(&PublishClock::timer_thread, this);
}

void
PublishClock::timer_thread()
{
  timer_->async_wait(boost::bind(&PublishClock::timer_callback, this));
  io_service_.run();
}

void
PublishClock::timer_callback()
{
  // calculated time
  ros_time_.clock += ros::Duration(playback_rate_ / rate_);
  publisher_->publish(ros_time_);

  // set next timer
  timer_->expires_at(timer_->expires_at() + boost::posix_time::milliseconds(1000. / rate_));
  timer_->async_wait(boost::bind(&PublishClock::timer_callback, this));
}
