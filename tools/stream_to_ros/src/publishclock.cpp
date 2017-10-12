#include "publishclock.h"

PublishClock::PublishClock(std::unique_ptr<ros::Publisher> publisher, float playback_rate, unsigned int rate)
  : publisher_( std::move(publisher) )
  , playback_rate_(playback_rate)
  , rate_(rate)
{
  // create callback timer
}
