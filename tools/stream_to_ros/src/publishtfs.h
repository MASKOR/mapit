#ifndef PUBLISHTFS_H
#define PUBLISHTFS_H

#include "publishtoros.h"

typedef std::map<double, std::unique_ptr<upns::tf::TransformStamped>> type_data;

class PublishTFs : public PublishToROS
{
public:
  using PublishToROS::PublishToROS;

  virtual void publish_entity(const std::string& entity_name, const std::shared_ptr<::Entity>& entity);

  void tf_timer_callback(const ros::TimerEvent&);
  void publish_one_tf_entity(std::unique_ptr<upns::tf::TransformStamped> tf_stamped);

private:
  std::shared_ptr<ros::Timer> tf_timer_;
  std::list<std::pair<type_data::iterator, type_data>> tf_entitys_;
};

#endif // PUBLISHTFS_H
