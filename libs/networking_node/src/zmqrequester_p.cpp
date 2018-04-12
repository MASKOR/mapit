/*******************************************************************************
 *
 * Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2015-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include "zmqrequester_p.h"
#include <mapit/msgs/services.pb.h>
#include <mapit/msgs/services_internal.pb.h>
#include <functional>

mapit::ZmqRequesterPrivate::ZmqRequesterPrivate(Repository *cache, std::string urlOutgoingRequests, bool operationsLocal)
    :ZmqProtobufNode( false ),
     m_cache( cache ),
     m_operationsLocal(operationsLocal)
{
    connect(urlOutgoingRequests);
    add_receivable_message_type<ReplyWorkspace>();
    add_receivable_message_type<ReplyEntitydata>();
    add_receivable_message_type<ReplyHierarchy>();
    add_receivable_message_type<ReplyListWorkspaces>();
    add_receivable_message_type<ReplyOperatorExecution>();
    add_receivable_message_type<ReplyStoreOperatorExecution>();
    add_receivable_message_type<ReplyGenericEntry>();
    add_receivable_message_type<ReplyStoreEntity>();
    add_receivable_message_type<ReplyDeleteEntity>();
    add_receivable_message_type<ReplyDeleteTree>();
    add_receivable_message_type<ReplyDoCommit>();
    add_receivable_message_type<ReplyCommit>();
}
