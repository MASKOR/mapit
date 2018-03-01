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

/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Tully Foote */

#include <upns/layertypes/tflayer/tf2/time_cache.h>
#include <upns/layertypes/tflayer/tf2/exceptions.h>

//#include "tf2/LinearMath/Transform.h"


using namespace mapit::tf2;


bool StaticCache::getData(mapit::time::Stamp time, TransformStorage & data_out, std::string* error_str) //returns false if data not available
{
  data_out = storage_;
  data_out.stamp_ = time;
  return true;
};

bool StaticCache::insertData(const TransformStorage& new_data)
{
  storage_ = new_data;
  return true;
};




void StaticCache::clearList() { return; };

unsigned long StaticCache::getListLength() {   return 1; };

CompactFrameID StaticCache::getParent(mapit::time::Stamp time, std::string* error_str)
{
  return storage_.frame_id_;
}

P_TimeAndFrameID StaticCache::getLatestTimeAndParent()
{
  return std::make_pair(mapit::time::Stamp(), storage_.frame_id_);
}

mapit::time::Stamp StaticCache::getLatestTimestamp()
{   
  return mapit::time::Stamp();
};

mapit::time::Stamp StaticCache::getOldestTimestamp()
{   
  return mapit::time::Stamp();
};

