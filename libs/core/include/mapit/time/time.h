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

#ifndef MAPIT_TIME_H
#define MAPIT_TIME_H

#include <mapit/msgs/datastructs.pb.h>
#include <chrono>
#include <ctime>
#include "date/date.h"

namespace std {
  namespace chrono {
    namespace date = ::date;
  }
}

namespace mapit
{
#if 0 // for autoindent
};
#endif

namespace time {
  using Clock = std::chrono::system_clock;

  using Stamp = std::chrono::time_point<Clock>;
  using Duration = std::chrono::duration<double>;

  using years = std::chrono::date::years;
  using month = std::chrono::date::months;
  using days = std::chrono::date::days;
  using hours = std::chrono::hours;
  using minutes = std::chrono::minutes;
  using seconds = std::chrono::seconds;
  using milliseconds = std::chrono::milliseconds;
  using microseconds = std::chrono::microseconds;
  using nanoseconds = std::chrono::nanoseconds;

  bool is_zero(Stamp stamp);

  Stamp from_sec_and_nsec(long sec, long nsec);

  Stamp from_msg(const mapit::msgs::Time& stamp);
  mapit::msgs::Time to_msg(const Stamp& stamp);

  void to_sec_and_nsec(Stamp stamp, unsigned long &sec, unsigned long &nsec);

  double to_sec(Stamp stamp);

  double to_sec(Duration dur);

  std::string to_string(Stamp stamp);
  std::string to_string_human(Stamp stamp);
}
}

#endif
