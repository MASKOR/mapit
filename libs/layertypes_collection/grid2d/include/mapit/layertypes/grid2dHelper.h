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
#include <mapit/msgs/datastructs.pb.h>

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

    /**
     * @brief Grid2DHelper::setFieldSize set size for data field
     * @param size_x width of field
     * @param size_y height of field
     */
    void setFieldSize(int size_x, int size_y);

    /**
     * @brief Grid2DHelper::setProbability sets a value to the datafield at positon (x,y)
     * @param x x-position
     * @param y y-position
     * @param probability value to be set, [0, 100] or [-1]
     */
    void setProbability(int x, int y, int probability);

    int getProbability(int x, int y);

    std::shared_ptr<mapit::msgs::Grid2D> getGrid();
    void setdGrid(std::shared_ptr<mapit::msgs::Grid2D> grid2D_data);

private:
   std::shared_ptr<mapit::msgs::Grid2D> m_Grid2D;

   /**
    * @brief Grid2DHelper::autoFieldExtender extends datafield if x or y or both are bigger than the current field
    * @param x new width
    * @param y new height
    */
   void autoFieldExtender(int x, int y);
};

#endif
