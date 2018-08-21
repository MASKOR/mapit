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
namespace mapit
{
namespace entitytypes
{
class Grid2DHelper
{
public:
    Grid2DHelper(const mapit::msgs::Grid2D &grid);
    Grid2DHelper();

    static const int GRID_UNKNOWN = -1;
    static const int GRID_OCCUPIED = 100;
    static const int GRID_FREE = 0;

    /**
     * @brief initGrid initialize grid with grid size, resolution and origin
     * @param size_x width in meter
     * @param size_y height in meter
     * @param resolution resolution in meter
     * @param origin origin position in real world
     */
    void initGrid(const float &size_x, const float &size_y,
                  const float &resolution, const mapit::msgs::Pose &origin);

    /**
     * @brief Grid2DHelper::setProbability sets a value to the datafield at positon (x,y)
     * @param x x-position
     * @param y y-position
     * @param probability value to be set, [0, 100] or [-1]
     */
    void setProbability(const float &x, const float &y, const int &probability);

    /**
     * @brief getProbability get the probalility value of the (x,y) position in the grid
     * @param x x-position
     * @param y y-position
     * @return the probability value
     */
    int getProbability(const float &x, const float &y);

    const mapit::msgs::Grid2D & getGrid();
    void setGrid(const msgs::Grid2D &grid);

private:
   std::shared_ptr<mapit::msgs::Grid2D> grid_;

//   /**
//    * @brief Grid2DHelper::autoFieldExtender extends datafield if x or y or both are bigger than the current field
//    * @param x new width
//    * @param y new height
//    */
//   void autoFieldExtender(const float &x, const float &y);

   /**
    * @brief getGridPosition calculates the fitted grid cell position
    * @param x x-pos in meter
    * @param y y-pos in meter
    * @return cell position in grid as int representation
    */
   unsigned int getGridPosition(const float &x, const float &y);

   /**
    * @brief getFittedXY Fits an x or y position in meter to the correct cell distance in the grid
    * @param xy x or y position in meter
    * @return fitted x or y grid cell number as int
    */
   unsigned int get_fitted_xy(const float &xy, const float &pose, const unsigned int &step);
   unsigned int get_fitted_x(const float &x);
   unsigned int get_fitted_y(const float &y);
};
}
}
#endif
