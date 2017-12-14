#ifndef PUBLISHPOINTCLOUDS_H
#define PUBLISHPOINTCLOUDS_H

#include "publishtoros.h"

class PublishPointClouds : public PublishToROS
{
public:
  using PublishToROS::PublishToROS;

  virtual void publish_entity(const std::string& entity_name, const std::shared_ptr<::Entity>& entity);
};

#endif // PUBLISHPOINTCLOUDS_H
