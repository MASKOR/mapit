/*******************************************************************************
 *
 * Copyright    2018 Tobias Neumann	<t.neumann@fh-aachen.de>
 *              2018 Michael Norget <mnorget@amt.rwth-aachen.de>
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

#ifndef Grid2DHELPER_H
#define Grid2DHELPER_H

#include <mapit/entitydata.h>
#include <mapit/operators/serialization/abstractentitydataprovider.h>
#include <mapit/msgs/datastructs.pb.h>
#include <pcl/point_types.h>
#include <mapit/layertypes/grid2d.h>

#ifdef _WIN32
#define MODULE_EXPORT __declspec(dllexport)
#else
#define MODULE_EXPORT // empty
#endif

class Grid2DHelper
{
public:
    Grid2DHelper(std::shared_ptr<mapit::msgs::Grid2D> grid2D_data);
    Grid2DHelper();

    void setFieldSize(int size_x, int size_y);

    void setMarker(int x, int y, int marker);

    int getMarker(int x, int y);

    std::shared_ptr<mapit::msgs::Grid2D> get2dGrid();
    void setdGrid(std::shared_ptr<mapit::msgs::Grid2D> grid2D_data);

private:
   std::shared_ptr<mapit::msgs::Grid2D> m_Grid2D;

   void autoFieldExtender(int x, int y);
   void initField();
};

#endif
