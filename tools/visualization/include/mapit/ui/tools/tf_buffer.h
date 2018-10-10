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

#ifndef VISUALIZATION_TFBUFFER_
#define VISUALIZATION_TFBUFFER_

#include <mutex>
#include <shared_mutex>

class QmlWorkspace;
namespace mapit {
namespace tf2 {
class BufferCore;
}
}

class TfBufferSingleton {
public:
    static TfBufferSingleton *get_instance();

    /**
     * @brief get_buffer_core_ro
     * @param workspace
     * @return tf2::BufferCore that is only allowed to lookup transforms
     */
    mapit::tf2::BufferCore *get_buffer_core_ro(QmlWorkspace* workspace);

    /**
     * @brief get_buffer_core gives complete and exclusive access to the buffer core
     * @param workspace
     * @return tf2:BufferCore that must be returned
     */
    std::unique_ptr<mapit::tf2::BufferCore> get_buffer_core(QmlWorkspace* workspace);

    /**
     * @brief return_buffer_core returns the exclusive access to the buffer core
     * @param tf_buffer
     */
    void return_buffer_core(std::unique_ptr<mapit::tf2::BufferCore> tf_buffer);

private:
    static std::mutex instance_mutex_;
    static TfBufferSingleton *instance_;
    TfBufferSingleton() : tf_buffer_(nullptr) { }

    /**
     * @brief create_tf_buffer checks if the tf_buffer needs to be created and does so
     *                         tf_buffer_mutex_ can not be locked before this method
     * @param workspace
     */
    void create_tf_buffer(QmlWorkspace* workspace);

    std::unique_ptr<mapit::tf2::BufferCore> tf_buffer_;
    mutable std::shared_mutex tf_buffer_mutex_;
};

#endif
