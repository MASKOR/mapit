#ifndef PUBLISHTFS_H
#define PUBLISHTFS_H

#include "publishtoros.h"

class PublishTFs : public PublishToROS
{
public:
  using PublishToROS::PublishToROS;

  virtual void publish_entity(std::shared_ptr<mapit::Entity> entity);
};

#endif // PUBLISHTFS_H
