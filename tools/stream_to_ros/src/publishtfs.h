/*******************************************************************************
 *
 * Copyright      2017 Tobias Neumann	<t.neumann@fh-aachen.de>
 *
******************************************************************************/

/*  This file is part of mapit.
 *
 *  Mapit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Mapit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
 */

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
