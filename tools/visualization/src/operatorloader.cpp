/*******************************************************************************
 *
 * Copyright      2017 Daniel Bulla	<d.bulla@fh-aachen.de>
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

#include "operatorloader.h"
#include <mapit/serialization/operatorlibrarymanager.h>
void OperatorLoader::run()
{
    QList<QVariant> result;
    std::vector<mapit::OperatorInfo> moduleInfos = mapit::OperatorLibraryManager::listOperators();
    for(auto iter = moduleInfos.cbegin(); iter != moduleInfos.cend() ; ++iter)
    {
        bool any = false;
        for(auto iter2=result.cbegin() ; iter2 != result.cend() ; ++iter2)
        {
            if(iter2->toJsonObject()["moduleName"].toString() == QString::fromStdString(iter->moduleName))
            {
                any = true;
                break;
            }
        }
        if(any) continue;

        QJsonObject obj;
        obj["compiler"] = QJsonValue(iter->compiler.c_str());
        obj["compilerConfig"] = QJsonValue(iter->compilerConfig.c_str());
        obj["date"] = QJsonValue(iter->date.c_str());
        obj["time"] = QJsonValue(iter->time.c_str());
        obj["moduleName"] = QJsonValue(iter->moduleName.c_str());
        obj["description"] = QJsonValue(iter->description.c_str());
        obj["author"] = QJsonValue(iter->author.c_str());
        obj["moduleVersion"] = iter->moduleVersion;
        obj["apiVersion"] = iter->apiVersion;
        obj["layerType"] = QJsonValue(iter->layerType.c_str());
        result.append(obj);
    }
    Q_EMIT operatorsAdded(result);
}

OperatorLoader::OperatorLoader(QObject *parent) : QThread(parent) {}

OperatorLoader::~OperatorLoader() {wait();}
