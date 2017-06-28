#ifndef MAPIT_TIME_H
#define MAPIT_TIME_H

#include <chrono>
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

class time {
public:
  using Clock = std::chrono::high_resolution_clock;

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

  static bool is_zero(Stamp stamp)
  {
    return stamp.time_since_epoch() == stamp.time_since_epoch().zero();
  }

  static Stamp from_sec_and_nsec(long sec, long nsec)
  {
    nanoseconds t_nsec(nsec);
    seconds t_sec(sec);

    Stamp t(t_sec + t_nsec);

    return t;
  }

  static void to_sec_and_nsec(Stamp stamp, long &sec, long &nsec)
  {
    seconds d_sec = std::chrono::duration_cast<seconds>( stamp.time_since_epoch() );
    sec = d_sec.count();

    Stamp s_nsec = stamp - d_sec;
    nsec = std::chrono::duration_cast<nanoseconds>( s_nsec.time_since_epoch() ).count();
  }

  static double to_sec(Stamp stamp)
  {
    Stamp full_seconds = std::chrono::date::floor<seconds>( stamp );
    double ns = std::chrono::duration_cast<nanoseconds>(stamp - full_seconds).count() * 0.000000001; //10**(-9)
    return std::chrono::duration_cast<seconds>( stamp.time_since_epoch() ).count() + ns;
  }

  static double to_sec(Duration dur)
  {
    return dur.count();
  }
};
}

#endif
