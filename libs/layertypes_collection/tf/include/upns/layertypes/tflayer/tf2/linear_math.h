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

#include <Eigen/Geometry>

Eigen::Translation3f
interpolate3(const Eigen::Translation3f& v0, const Eigen::Translation3f& v1, float rt)
{
  float s = float(1.0) - rt;
  Eigen::Translation3f o(
          s * v0.x() + rt * v1.x(),
          s * v0.y() + rt * v1.y(),
          s * v0.z() + rt * v1.z()
        );

  return o;
}

Eigen::Vector3f
quatRotate(const Eigen::Quaternionf& rotation, const Eigen::Vector3f& v)
{
    Eigen::Quaternionf q(
          -rotation.x() * v.x() - rotation.y() * v.y() - rotation.z() * v.z()
        , rotation.w() * v.x() + rotation.y() * v.z() - rotation.z() * v.y()
        , rotation.w() * v.y() + rotation.z() * v.x() - rotation.x() * v.z()
        , rotation.w() * v.z() + rotation.x() * v.y() - rotation.y() * v.x()
        );
    q *= rotation.inverse();
    return Eigen::Vector3f(q.x(),q.y(),q.z());
}
