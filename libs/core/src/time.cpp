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
};
}
