/*******************************************************************************
 *
 * Copyright      2018 Tobias Neumann <t.neumann@fh-aachen.de>
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

#include "mapit/ui/tools/tf_buffer.h"

#include <mapit/layertypes/tflayer/tf2/buffer_core.h>

#include <mapit/ui/bindings/qmlworkspace.h>

std::mutex TfBufferSingleton::instance_mutex_;
TfBufferSingleton *TfBufferSingleton::instance_;

TfBufferSingleton *
TfBufferSingleton::get_instance()
{
    std::unique_lock<std::mutex> lock(instance_mutex_);
    if ( ! instance_ ) {
        instance_ = new TfBufferSingleton();
    }
    return instance_;
}

void
TfBufferSingleton::create_tf_buffer(QmlWorkspace* workspace)
{
    std::unique_lock<std::shared_mutex> lock(tf_buffer_mutex_);

    if ( ! tf_buffer_ ) {
        tf_buffer_ = std::make_unique<mapit::tf2::BufferCore>(workspace->getWorkspaceObj().get(), "");;
    }
}

mapit::tf2::BufferCore *
TfBufferSingleton::get_buffer_core_ro(QmlWorkspace* workspace)
{
    create_tf_buffer(workspace);

    std::shared_lock<std::shared_mutex> lock(tf_buffer_mutex_);
    return tf_buffer_.get();
}

std::unique_ptr<mapit::tf2::BufferCore>
TfBufferSingleton::get_buffer_core(QmlWorkspace* workspace)
{
    create_tf_buffer(workspace);

    tf_buffer_mutex_.lock();
    return std::move(tf_buffer_);
}

void
TfBufferSingleton::return_buffer_core(std::unique_ptr<mapit::tf2::BufferCore> tf_buffer)
{
    tf_buffer_ = std::move(tf_buffer);
    tf_buffer_mutex_.unlock();
}
