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
