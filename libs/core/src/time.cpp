/*******************************************************************************
 *
 * Copyright 2017-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include <mapit/time/time.h>

namespace mapit
{
#if 0 // for autoindent
};
#endif

namespace  time {

  bool is_zero(Stamp stamp)
  {
    return stamp.time_since_epoch() == stamp.time_since_epoch().zero();
  }

  Stamp from_sec_and_nsec(long sec, long nsec)
  {
    nanoseconds t_nsec(nsec);
    seconds t_sec(sec);

    Stamp t(t_sec + t_nsec);

    return t;
  }

  Stamp from_msg(const mapit::msgs::Time& stamp)
  {
      return from_sec_and_nsec(stamp.sec(), stamp.nsec());
  }

  mapit::msgs::Time to_msg(const Stamp& stamp)
  {
      unsigned long sec, nsec;
      to_sec_and_nsec(stamp, sec, nsec);
      mapit::msgs::Time msg;
      msg.set_sec(sec);
      msg.set_nsec(nsec);
      return msg;
  }

  void to_sec_and_nsec(Stamp stamp, unsigned long &sec, unsigned long &nsec)
  {
    seconds d_sec = std::chrono::duration_cast<seconds>( stamp.time_since_epoch() );
    sec = d_sec.count();

    Stamp s_nsec = stamp - d_sec;
    nsec = std::chrono::duration_cast<nanoseconds>( s_nsec.time_since_epoch() ).count();
  }

  double to_sec(Stamp stamp)
  {
    Stamp full_seconds = std::chrono::date::floor<seconds>( stamp );
    double ns = std::chrono::duration_cast<nanoseconds>(stamp - full_seconds).count() * 0.000000001; //10**(-9)
    return std::chrono::duration_cast<seconds>( stamp.time_since_epoch() ).count() + ns;
  }

  double to_sec(Duration dur)
  {
    return dur.count();
  }

  std::string to_string(Stamp stamp)
  {
    unsigned long sec, nsec;
    to_sec_and_nsec(stamp, sec, nsec);

    std::string nsec_str = std::to_string(nsec);
    int zeros_to_add = 9 - nsec_str.size();
    std::string zeroes = "";
    for (int i = 0; i < zeros_to_add; ++i) {
        zeroes += "0";
    }

    std::string out = std::to_string(sec) + "." + zeroes + nsec_str;
    return out;
  }
};
}
