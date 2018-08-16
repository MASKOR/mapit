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

Grid2DHelper::Grid2DHelper() {

}

Grid2DHelper::Grid2DHelper(std::shared_ptr<mapit::msgs::Grid2D> grid2D_data) {
    this->m_Grid2D = grid2D_data;
}



/**
 * @brief Grid2DHelper::setFieldSize set size for data field
 * @param size_x width of field
 * @param size_y height of field
 */
void Grid2DHelper::setFieldSize(int size_x, int size_y) {
    m_Grid2D->set_width(size_x);
    m_Grid2D->set_height(size_y);
}

/**
 * @brief Grid2DHelper::setMarker sets a value to the datafield at positon (x,y)
 * @param x x-position
 * @param y y-position
 * @param marker value to be set, [0, 100] or [-1]
 */
void Grid2DHelper::setMarker(int x, int y, int marker) {
    if (x > m_Grid2D->width() || y > m_Grid2D->height()) {
        log_error("set value outside data field");
        return;
        // ERROR
    }
    std::string data = m_Grid2D->data();
    if(data.empty()) {
        initField();
    }

    // set value
    data[y * m_Grid2D->width() + x] = (signed char)marker;
    m_Grid2D->set_allocated_data(&data);
}

int Grid2DHelper::getMarker(int x, int y) {
    if (x > m_Grid2D->width() || y > m_Grid2D->height()) {
        log_error("Position outside of grid");
        // Error? Or just return -1 for unknown?
    }
    std::string field = m_Grid2D->data();
    if (field.empty()) { //gid size set but not inizialized
        return -1; //
    }
    return (int)field[y * m_Grid2D->height() + x];
}

std::shared_ptr<mapit::msgs::Grid2D> Grid2DHelper::get2dGrid() {
    return m_Grid2D;
}

void Grid2DHelper::setdGrid(std::shared_ptr<mapit::msgs::Grid2D> grid2D_data) {
    m_Grid2D = grid2D_data;
}

void Grid2DHelper::initField() {
    signed char defVal = -1; // -1 == unknown
    std::string data;

    // init field in row-major order
    for (int y = 0; y < m_Grid2D->height(); y++) {
        for (int x = 0; x < m_Grid2D->width(); x++) {
            data += defVal;
        }
    }

    m_Grid2D->set_allocated_data(&data);
}

/**
 * @brief Grid2DHelper::autoFieldExtender extends datafield if x or y or both are bigger than the current field
 * @param x new width
 * @param y new height
 */
void Grid2DHelper::autoFieldExtender(int x, int y) {
    std::string data = m_Grid2D->data();
    if (x > m_Grid2D->width()) { //extend columns
        std::string newData;
        signed char defVal = -1;
        // extend field and add defVals for new entries
        for (int yc = 0; yc < m_Grid2D->height(); yc++) {
            for (int xc = 0; xc < x; xc++) {
                if (xc < m_Grid2D->width()) {
                    //newData += data[yc*m_Grid2D->width() + xc];
                    newData += getMarker(xc, yc);
                } else {
                    newData += defVal;
                }
            }
        }
        m_Grid2D->set_width(x); // set new width
        // m_Grid2D->set_allocated_data(&newData);
        data = newData;
    }

    if (y > m_Grid2D->height()) { // extend height
        signed char defVal = -1;
        // extend field and add defVals for new entries
        // this is easy, just add rows at the end of the string
        // no copy action required
        for (int yc = m_Grid2D->height(); yc < y; yc++) {
            for (int xc = 0; xc < x; xc++) {
                data += defVal;
            }
        }
        m_Grid2D->set_height(y); // set new height
    }
}
