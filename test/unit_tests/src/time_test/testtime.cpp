#include "testtime.h"
#include "../../src/autotest.h"

#include <mapit/time/time.h>

#include <typeinfo>
#include <iostream>

void TestTime::init()
{
}

void TestTime::cleanup()
{
}

void TestTime::initTestCase()
{
}

void TestTime::cleanupTestCase()
{
}

void TestTime::testTime()
{
  namespace t = mapit::time;

  t::Stamp now = t::Clock::now();
  t::Stamp d_5_sec_ago = now - t::Duration(5);

  // random time, out of ROS at the time of writing this test
  long ros_sec = 1497347909;
  long ros_nsec = 336102938;
  // the following are 1497347909 converted from unix time (unix time is what ros uses)
  long ros_year = 2017;
  long ros_month = 6;
  long ros_day = 13;
  long ros_hour = 9;
  long ros_minute = 58;
  long ros_second = 29;

  t::Stamp ros = t::from_sec_and_nsec(ros_sec, ros_nsec);
  long sec_new;
  long nsec_new;
  t::to_sec_and_nsec(ros, sec_new, nsec_new);

  QVERIFY(sec_new == ros_sec);
  QVERIFY(nsec_new == ros_nsec);

  QVERIFY(t::only_year(ros) == ros_year);
  QVERIFY(t::only_month(ros) == ros_month);
  QVERIFY(t::only_day(ros) == ros_day);
  QVERIFY(t::only_hours(ros) == ros_hour);
  QVERIFY(t::only_minutes(ros) == ros_minute);
  QVERIFY(t::only_seconds(ros) == ros_second);
}

DECLARE_TEST(TestTime)
