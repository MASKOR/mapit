/*******************************************************************************
 *
 * Copyright 2017-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include <mapit/operators/module.h>
#include <mapit/logging.h>
#include <mapit/operators/versioning/workspacewritable.h>
#include <mapit/operators/operationenvironment.h>
#include <mapit/errorcodes.h>
#include <string>

#include <mapit/layertypes/tflayer.h>
#include <mapit/layertypes/tflayer/utils.h>
#include <mapit/layertypes/tflayer/tf2/buffer_core.h>

#include "reg_local.h"

mapit::StatusCode operate_reg_local_icp(mapit::OperationEnvironment* env)
{
    /** structure:
     * {
     *  <string>"tf-prefix" : ..., // the prefix where to look for transforms (default "")
     *  <string>"input"[] : ..., // can either be, a list of entities or a tree containing enteties
     *  <string>"target" : ...,
     *
     *  optional <string>"frame_id" : ..., // all data given to ICP will be in this frame
     *  optional <bool>"use-metascan" : ..., //
     *
     *  <enum-as-string>"handle-result" : ["tf-add", "tf-combine", "data-change"],
     *  optional <string>"tf-frame_id" : ..., // in case of tf change or add
     *  optional <string>"tf-child_frame_id" : ..., // in case of tf change or add
     *  optional <bool>"tf-is_static" : ..., // in case of tf change or add (true only works with one input specified) (default false)
     *
     *  <enum-as-string>"matching-algorithm" : ["icp"], // what kind of local matching algorithm should be used
     *
     *  optional <int>icp-maximum-iterations: ...,
     *  optional <double>icp-max-correspondence-distance: ...,
     *  optional <double>icp-transformation-epsilon: ...,
     * }
     *
     * comments:
     * - tf-add:      will add the results of the ICP without checking anything
     *                (old transforms (between "tf-frame_id" and "tf-child_frame_id")
     *                should probably been removed beforehand)
     * - tf-combine:  will combine the transforms in the system with the transforms
     *                from ICP but only at the times of ICP.
     *                That means afterwards the transforms between "tf-frame_id"
     *                and "tf-child_frame_id" are changed. In the time between the
     *                pointclouds given in the parameter "input" there will only
     *                be transforms at the time of each cloud (even so there was
     *                more transforms beforehand) these transforms will be the
     *                combination of the result of ICP and the previous transform
     *                at this time. Before and after the time of the pointclouds
     *                from "input" there will be no change.
     *                E.g. there are transforms at the times [0, 10, 20, 30, 40, 50],
     *                the stampes of the pointclouds in "input" are [19, 20, 31].
     *                This means the new transforms are [0, 10, 19-1ns, 19, 20, 31,
     *                31+1ns, 40, 50]. The transforms at 0, 10, 40 and 50 are the
     *                old transforms. The transforms at 19, 20, 31 are the new
     *                transforms * old transforms. And the transforms at 19-1ns
     *                and 31+1ns are the interpolated transforms based on the
     *                old transforms.
     *                This way, the old transformes are used from 0 - 19-1ns and
     *                31+1ns - 50. And the new transforms from 19 - 31.
     * - data-change: will change the data of the "input" clouds, tfs are not changed.
     */

    mapit::StatusCode status;
    mapit::RegLocal reg_local(env, status);
    if ( ! mapitIsOk( status ) ) {
        return status;
    }
    return reg_local.operate();
}

MAPIT_MODULE(OPERATOR_NAME, "execute local scanmatching on pointclouds", "fhac", OPERATOR_VERSION, "any", true, &operate_reg_local_icp)
