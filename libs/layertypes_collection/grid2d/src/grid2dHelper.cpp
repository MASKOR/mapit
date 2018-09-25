/*******************************************************************************
 *
 * Copyright 2018 Michael Norget mnorget@amt.rwth-aachen.de
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

#include "mapit/layertypes/grid2dHelper.h"
#include <mapit/logging.h>
#include <sstream>
#include <stdexcept>
#include <cmath>

using namespace mapit::entitytypes;

Grid2DHelper::Grid2DHelper()
{

}

Grid2DHelper::Grid2DHelper(const msgs::Grid2D &grid)
{
    grid_ = std::make_shared<mapit::msgs::Grid2D>(grid);
}



void
Grid2DHelper::initGrid(const float &size_x, const float &size_y,
                            const float &resolution, const msgs::Pose &origin)
{
    grid_ = std::make_shared<mapit::msgs::Grid2D>();
    grid_->set_width( static_cast<unsigned int>(size_x / resolution) );
    grid_->set_height( static_cast<unsigned int>(size_y / resolution) );

    grid_->set_resolution(resolution);
    *grid_->mutable_origin() = origin;

    grid_->mutable_data()->resize(grid_->width() * grid_->height(), -1);
}

void
Grid2DHelper::setProbability(const float &x, const float &y, const int &probability)
{
    // set value
    grid_->mutable_data()->at( getGridPosition(x, y) ) = static_cast<signed char>(probability);
}

int
Grid2DHelper::getProbability(const float &x, const float &y)
{
    return static_cast<int>(grid_->data().at( getGridPosition(x, y) ));
}

const mapit::msgs::Grid2D &
Grid2DHelper::getGrid()
{
    return *grid_.get();
}

void
Grid2DHelper::setGrid(const msgs::Grid2D &grid)
{
    grid_ = std::make_shared<msgs::Grid2D>(grid);
}


unsigned int
Grid2DHelper::getGridPosition(const float &x, const float &y)
{
    // x and y in meters
    unsigned int xPos = get_fitted_x(x);
    unsigned int yPos = get_fitted_y(y);

    if (   xPos > grid_->width()
        || yPos > grid_->height()) {
        log_error("Grid2DHelper: Position outside of grid: x=" << x << "/" << xPos << "/" << grid_->width()
                  << " y=" << y << "/" << yPos << "/" << grid_->height());
        throw std::out_of_range("Grid2DHelper: position is out of field boundaries");
    }
//    unsigned int pos = yPos * grid_->width() + xPos;
//    if (pos > grid_->data().length()) { //This should never happen
//        log_error("Grid2DHelper: extra check -> Position outside of grid: x=" << x << "/" << xPos
//                  << " y=" << y << "/" << yPos << "; at pos " << pos  << " for length " << grid_->data().length());
//        throw std::out_of_range("Grid2DHelper: extra check -> position is out of field boundaries");
//    }
    return yPos * grid_->width() + xPos;
}

unsigned int
Grid2DHelper::get_fitted_xy(const float &xy, const float &pose, const unsigned int &step)
{
    return static_cast<unsigned int>(
                  std::roundf( xy / grid_->resolution() )       // offset from pose on grid
                + std::roundf( pose / grid_->resolution() )     // offset of pose on grid
                + std::roundf( step / 2 )                       // center of grid on respective axis
                );
}

unsigned int
Grid2DHelper::get_fitted_x(const float &x)
{
    return get_fitted_xy(x, grid_->origin().translation().x(), grid_->width());
}

unsigned int
Grid2DHelper::get_fitted_y(const float &y)
{
    return get_fitted_xy(y, grid_->origin().translation().y(), grid_->height());
}


//void
//Grid2DHelper::autoFieldExtender(const float &x, const float &y)
//{
//    std::string data = grid_->data();
//    if (x > grid_->width()) { //extend columns
//        std::string newData;
//        signed char defVal = -1;
//        // extend field and add defVals for new entries
//        for (unsigned int yc = 0; yc < grid_->height(); yc++) {
//            for (unsigned int xc = 0; xc < x; xc++) {
//                if (xc < grid_->width()) {
//                    newData += static_cast<signed char>(getProbability(xc, yc));
//                } else {
//                    newData += defVal;
//                }
//            }
//        }
//        grid_->set_width(get_fitted_xy(x)); // set new width
//        data = newData;
//    }

//    if (y > grid_->height()) { // extend height
//        signed char defVal = -1;
//        // extend field and add defVals for new entries
//        // this is easy, just add rows at the end of the string
//        // no copy action required
//        for (unsigned int yc = grid_->height(); yc < y; yc++) {
//            for (int xc = 0; xc < x; xc++) {
//                data += defVal;
//            }
//        }
//        grid_->set_height(get_fitted_xy(y)); // set new height
//    }
//}
