#ifndef PUBLISHPOINTCLOUDS_H
#define PUBLISHPOINTCLOUDS_H

#include "publishtoros.h"

class PublishPointClouds : public PublishToROS
{
public:
  using PublishToROS::PublishToROS;

  virtual void publish_entity(std::shared_ptr<mapit::Entity> entity);
};

#endif // PUBLISHPOINTCLOUDS_H
