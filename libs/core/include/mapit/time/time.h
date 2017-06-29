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

namespace time {
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

  bool is_zero(Stamp stamp);

  Stamp from_sec_and_nsec(long sec, long nsec);

  void to_sec_and_nsec(Stamp stamp, long &sec, long &nsec);

  double to_sec(Stamp stamp);

  double to_sec(Duration dur);
}
}

#endif
