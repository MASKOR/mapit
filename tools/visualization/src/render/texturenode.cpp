/*******************************************************************************
 *
 * Copyright      2015 Daniel Bulla	<d.bulla@fh-aachen.de>
 *                2015 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include "texturenode.h"

TextureNode::TextureNode(QQuickWindow *window)
    : m_id(0)
    , m_size(0, 0)
    , m_texture(0)
    , m_window(window)
{
    // Our texture node must have a texture, so use the default 0 texture.
    m_texture = m_window->createTextureFromId(0, QSize(1, 1));
    setTexture(m_texture);
    setFiltering(QSGTexture::Linear);
}

TextureNode::~TextureNode()
{
    delete m_texture;
}

void TextureNode::newTexture(int id, const QSize &size) {
    m_mutex.lock();
    m_id = id;
    m_size = size;
    m_mutex.unlock();

    // We cannot call QQuickWindow::update directly here, as this is only allowed
    // from the rendering thread or GUI thread.
    Q_EMIT pendingNewTexture();
}

void TextureNode::prepareNode() {
    m_mutex.lock();
    int newId = m_id;
    QSize size = m_size;
    m_id = 0;
    m_mutex.unlock();
    if (newId) {
        delete m_texture;
        // note: include QQuickWindow::TextureHasAlphaChannel if the rendered content
        // has alpha.
        m_texture = m_window->createTextureFromId(newId, size);
        setTexture(m_texture);

        markDirty(DirtyMaterial);

        // This will notify the rendering thread that the texture is now being rendered
        // and it can start rendering to the other one.
        Q_EMIT textureInUse();
    }
}
