/*******************************************************************************
 *
 * Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2015-2017 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include "upns/ui/bindings/qmltree.h"

QmlTree::QmlTree(QObject *parent)
    :QObject(parent), m_tree( nullptr )
{

}

QmlTree::QmlTree(std::shared_ptr<mapit::msgs::Tree> &tree, QObject *parent)
    :QObject(parent), m_tree(tree)
{

}

QStringList QmlTree::getRefs()
{
    if(!m_tree) return QStringList();
    QStringList refs;
    for(google::protobuf::Map<std::string, mapit::msgs::ObjectReference >::const_iterator iter(m_tree->refs().cbegin());
        iter != m_tree->refs().cend();
        ++iter)
    {
        refs.append(QString::fromStdString(iter->first));
    }
    return refs;
}

QString QmlTree::oidOfRef(QString name)
{
    if(!m_tree) return "";
    return QString::fromStdString(m_tree->refs().at(name.toStdString()).id());
}

bool QmlTree::isValid() const
{
    return m_tree != nullptr;
}
