#ifndef MAPIT_TIME_H
#define MAPIT_TIME_H

#include <chrono>
#include "date/date.h"

namespace mapit
{
#if 0 // for autoindent
};
#endif

namespace time {
  namespace c_ = std::chrono;
  namespace date = ::date;
  using Clock = c_::high_resolution_clock;

  using Duration = c_::duration<long>;

  using Stamp = c_::time_point<Clock>;

  using years = date::years;
  using month = date::months;
  using days = date::days;
  using hours = c_::hours;
  using minutes = c_::minutes;
  using seconds = c_::seconds;
  using milliseconds = c_::milliseconds;
  using microseconds = c_::microseconds;
  using nanoseconds = c_::nanoseconds;

  Stamp from_sec_and_nsec(long sec, long nsec)
  {
    nanoseconds t_nsec(nsec);
    seconds t_sec(sec);

    Stamp t(t_sec + t_nsec);

    return t;
  }
  void to_sec_and_nsec(Stamp stamp, long &sec, long &nsec)
  {
    Duration d_sec = c_::duration_cast<seconds>( stamp.time_since_epoch() );
    sec = d_sec.count();
    Stamp s_nsec = stamp - d_sec;
    nsec = c_::duration_cast<nanoseconds>( s_nsec.time_since_epoch() ).count();
  }

  unsigned int only_year(Stamp stamp)
  {
    date::year_month_day ymd = date::floor<days>( stamp );
    return int(ymd.year());
  }

  unsigned int only_month(Stamp stamp)
  {
    date::year_month_day ymd = date::floor<days>( stamp );
    return unsigned(ymd.month());
  }

  unsigned int only_day(Stamp stamp)
  {
    date::year_month_day ymd = date::floor<days>( stamp );
    return unsigned(ymd.day());
  }

  unsigned int only_hours(Stamp stamp)
  {
    Stamp full_days = date::floor<date::days>( stamp );
    return c_::duration_cast<hours>( stamp - full_days ).count();
  }

  unsigned int only_minutes(Stamp stamp)
  {
    Stamp full_hours = date::floor<hours>( stamp );
    return c_::duration_cast<minutes>( stamp - full_hours ).count();
  }

  unsigned int only_seconds(Stamp stamp)
  {
    Stamp full_minutes = date::floor<minutes>( stamp );
    return c_::duration_cast<seconds>( stamp - full_minutes ).count();
  }
}
}

#endif
