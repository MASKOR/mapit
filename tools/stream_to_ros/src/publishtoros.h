#ifndef PUBLISHTOROS_H
#define PUBLISHTOROS_H

#include <upns/versioning/checkout.h>
#include <upns/logging.h>
#include <mapit/time/time.h>

#include <ros/ros.h>
#include <std_msgs/Header.h>

struct Entity {
    std::string user_prefix;
    std::shared_ptr<mapit::msgs::Entity> entity;
    std::string entity_name;
};

class PublishToROS
{
public:
  PublishToROS(std::shared_ptr<upns::Checkout> checkout, std::shared_ptr<ros::NodeHandle> node_handle, std::unique_ptr<ros::Publisher> publisher)
    : checkout_(checkout)
    , node_handle_(node_handle)
    , publisher_( std::move(publisher) )
  { }

  void add_entity(::Entity entity)
  {
      // add to list
      double time = mapit::time::to_sec( mapit::time::from_sec_and_nsec( entity.entity->stamp().sec(), entity.entity->stamp().nsec() ) );
      entities_.insert(std::pair<double, std::shared_ptr<::Entity>>(time, std::make_shared<::Entity>(entity)));
      offset_ = -1;
  }

  virtual void publish_entity(const std::string& entity_name, const std::shared_ptr<::Entity>& entity) = 0;

  double get_stamp_of_first_data()
  {
    if ( entities_.empty() ) {
      return -1;
    } else {
      log_info("time is: " + std::to_string(entities_.begin()->first));
      return entities_.begin()->first;
    }
  }

  void set_offset(double offset)
  {
    offset_ = offset;
    log_info("use time: " + std::to_string( offset_ ));
  }

  void start_publishing()
  {
    entity_next_ = entities_.begin();
    timer_ = std::make_shared<ros::Timer>( node_handle_->createTimer(ros::Rate(100), &PublishToROS::timer_callback, this) );
  }

protected:
  void timer_callback(const ros::TimerEvent&)
  {
    // check for end
    if (entity_next_ == entities_.end()) {
      log_warn("reached end of entities \"" + entities_.begin()->second->user_prefix + "\"");
      timer_ = nullptr;
      return;
    }

    // for all where the time is over
    double time_ros = ros::Time::now().toSec();
    for (
         ;    entity_next_ != entities_.end()
           && entity_next_->first <= time_ros - offset_
         ; ++entity_next_) {
      std::shared_ptr<::Entity> entity = entity_next_->second;
      publish_entity(entity->entity_name, entity);
    }
  }

  std_msgs::Header get_header(std::shared_ptr<mapit::msgs::Entity> entity)
  {
    std_msgs::Header header;

    if (0 == entity->frame_id().compare("")) {
      header.frame_id = _DEFAULT_FRAME_ID_;
    } else {
      header.frame_id = entity->frame_id();
    }

    ros::Time et;
    et.fromSec( mapit::time::to_sec( mapit::time::from_sec_and_nsec(entity->stamp().sec(), entity->stamp().nsec()) ) );
    header.stamp = et;

    return header;
  }

  const std::string _DEFAULT_FRAME_ID_ = "world";
  std::shared_ptr<upns::Checkout> checkout_;
  std::unique_ptr<ros::Publisher> publisher_;

  std::shared_ptr<ros::NodeHandle> node_handle_;
  double offset_;
private:
  std::shared_ptr<ros::Timer> timer_;
  std::map<double, std::shared_ptr<::Entity>> entities_;
  std::map<double, std::shared_ptr<::Entity>>::iterator entity_next_;
};

#endif // PUBLISHTOROS_H
