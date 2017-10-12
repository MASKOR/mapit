#ifndef PUBLISHTOROS_H
#define PUBLISHTOROS_H

#include <upns/versioning/checkout.h>

#include <ros/ros.h>
#include <std_msgs/Header.h>

class PublishToROS
{
public:
  PublishToROS(std::shared_ptr<upns::Checkout> checkout, std::shared_ptr<ros::NodeHandle> node_handle, std::unique_ptr<ros::Publisher> publisher)
    : checkout_(checkout)
    , node_handle_(node_handle)
    , publisher_( std::move(publisher) )
  { }

  PublishToROS(std::shared_ptr<upns::Checkout> checkout, std::shared_ptr<ros::NodeHandle> node_handle, std::unique_ptr<ros::Publisher> publisher, std::shared_ptr<mapit::Layer> layer)
    : PublishToROS(checkout, node_handle, std::move(publisher))
  {
    double now = ros::Time::now().toSec(); // ROS time is used, because the data is played back in ROS time

    // create a time sorted map of all entities
    for (auto entity : checkout_->getListOfEntities(layer)) {
      double time = mapit::time::to_sec( entity->stamp() );
      layer_.insert(std::pair<double, std::shared_ptr<mapit::Entity>>(time, entity));
    }
    if ( layer_.empty() ) {
      entity_next_ = layer_.end();
    } else {
      entity_next_ = layer_.begin()++;

      double first = mapit::time::to_sec( entity_next_->second->stamp() );
      offset_ = now - first;

      // create a timer for the 1. entity
      timer_ = std::make_shared<ros::Timer>( node_handle_->createTimer(ros::Rate(100), &PublishToROS::timer_callback, this) );
    }
  }

  virtual void publish_entity(std::shared_ptr<mapit::Entity> entity) = 0;

protected:
  void timer_callback(const ros::TimerEvent&)
  {
    // check for end
    if (entity_next_ == layer_.end()) {
//      log_info("reached end of layer");
      timer_ = nullptr;
      return;
    }

    // for all where the time is over
    double time_ros = ros::Time::now().toSec();
    for (
         ;    entity_next_ != layer_.end()
           && entity_next_->first <= time_ros - offset_
         ; ++entity_next_) {
      std::shared_ptr<mapit::Entity> entity = entity_next_->second;
      publish_entity(entity);
    }
  }

  std_msgs::Header get_header(std::shared_ptr<mapit::Entity> entity)
  {
    std_msgs::Header header;

    if (0 == entity->frame_id().compare("")) {
      header.frame_id = _DEFAULT_FRAME_ID_;
    } else {
      header.frame_id = entity->frame_id();
    }

    ros::Time et;
    et.fromSec( mapit::time::to_sec( entity->stamp() ) );
    header.stamp = et;

    return header;
  }

  const std::string _DEFAULT_FRAME_ID_ = "world";
  std::shared_ptr<upns::Checkout> checkout_;
  std::unique_ptr<ros::Publisher> publisher_;

private:
  double offset_;
  std::shared_ptr<ros::Timer> timer_;
  std::map<double, std::shared_ptr<mapit::Entity>> layer_;
  std::map<double, std::shared_ptr<mapit::Entity>>::iterator entity_next_;

  std::shared_ptr<ros::NodeHandle> node_handle_;
};

#endif // PUBLISHTOROS_H
