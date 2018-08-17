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

using namespace mapit::entitytypes;

Grid2DHelper::Grid2DHelper()
{

}

Grid2DHelper::Grid2DHelper(std::shared_ptr<mapit::msgs::Grid2D> grid2D_data)
{
    this->m_Grid2D = grid2D_data;
}



void Grid2DHelper::initGrid(const unsigned int &size_x, const unsigned int &size_y,
                  const unsigned int &resolution, mapit::msgs::Pose origin)
{
    m_Grid2D->set_width(size_x);
    m_Grid2D->set_height(size_y);

    m_Grid2D->set_resolution(resolution);
    m_Grid2D->set_allocated_origin(&origin);

    m_Grid2D->mutable_data()->resize(m_Grid2D->width() * m_Grid2D->height(), -1);
}

void Grid2DHelper::setProbability(const float &x, const float &y, int &probability)
{
    if (getFittedXY(x) > m_Grid2D->width() || getFittedXY(y) > m_Grid2D->height()) {
        log_error("Grid2DHelper: set value outside grid");
        throw std::out_of_range("Grid2DHelper: position is out of field boundaries");
    }
    // set value
    m_Grid2D->mutable_data()[getGridPosition(x, y)] = static_cast<signed char>(probability);
}

int Grid2DHelper::getProbability(const float &x, const float &y)
{
    if (getFittedXY(x) > m_Grid2D->width() || getFittedXY(y) > m_Grid2D->height()) {
        log_error("Grid2DHelper: Position outside of grid");
        throw std::out_of_range("Grid2DHelper: position is out of field boundaries");
    }
    return static_cast<int>(m_Grid2D->data()[getGridPosition(x, y)]);
}

std::shared_ptr<mapit::msgs::Grid2D> Grid2DHelper::getGrid()
{
    return m_Grid2D;
}

void Grid2DHelper::setGrid(std::shared_ptr<mapit::msgs::Grid2D> grid2D_data)
{
    m_Grid2D = grid2D_data;
}


unsigned int Grid2DHelper::getGridPosition(const float &x, const float &y)
{
    unsigned int xPos = getFittedXY(x);
    unsigned int yPos = getFittedXY(y);
    return yPos * m_Grid2D->height() + xPos;
}

unsigned int Grid2DHelper::getFittedXY(const float &xy)
{
    return static_cast<unsigned int>(std::roundf(xy/m_Grid2D->resolution()));
}

void Grid2DHelper::autoFieldExtender(const float &x, const float &y)
{
    std::string data = m_Grid2D->data();
    if (x > m_Grid2D->width()) { //extend columns
        std::string newData;
        signed char defVal = -1;
        // extend field and add defVals for new entries
        for (unsigned int yc = 0; yc < m_Grid2D->height(); yc++) {
            for (unsigned int xc = 0; xc < x; xc++) {
                if (xc < m_Grid2D->width()) {
                    newData += static_cast<signed char>(getProbability(xc, yc));
                } else {
                    newData += defVal;
                }
            }
        }
        m_Grid2D->set_width(getFittedXY(x)); // set new width
        data = newData;
    }

    if (y > m_Grid2D->height()) { // extend height
        signed char defVal = -1;
        // extend field and add defVals for new entries
        // this is easy, just add rows at the end of the string
        // no copy action required
        for (unsigned int yc = m_Grid2D->height(); yc < y; yc++) {
            for (int xc = 0; xc < x; xc++) {
                data += defVal;
            }
        }
        m_Grid2D->set_height(getFittedXY(y)); // set new height
    }
}
