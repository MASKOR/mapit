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
  // random time, out of ROS at the time of writing this test
  long ros_sec = 1497347909;
  long ros_nsec = 336102938;
  double ros_as_double = ros_sec + (ros_nsec * 0.000000001);
  // the following are 1497347909 converted from unix time (unix time is what ros uses)
  long ros_year = 2017;
  long ros_month = 6;
  long ros_day = 13;
  long ros_hour = 9;
  long ros_minute = 58;
  long ros_second = 29;

  // test import and export
  mapit::time::Stamp ros = mapit::time::from_sec_and_nsec(ros_sec, ros_nsec);
  long sec_new;
  long nsec_new;
  mapit::time::to_sec_and_nsec(ros, sec_new, nsec_new);

  QVERIFY(sec_new == ros_sec);
  QVERIFY(nsec_new == ros_nsec);

  // test duration
  mapit::time::Duration ros_d = std::chrono::duration_cast<mapit::time::Duration>( ros.time_since_epoch() );

  QVERIFY(ros_as_double == ros_d.count());

//  QVERIFY(mapit::time::only_year(ros) == ros_year);
//  QVERIFY(mapit::time::only_month(ros) == ros_month);
//  QVERIFY(mapit::time::only_day(ros) == ros_day);
//  QVERIFY(mapit::time::only_hours(ros) == ros_hour);
//  QVERIFY(mapit::time::only_minutes(ros) == ros_minute);
//  QVERIFY(mapit::time::only_seconds(ros) == ros_second);
}

DECLARE_TEST(TestTime)
