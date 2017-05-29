#include "upns/ui/bindings/qmlplymeshgeometry.h"
#include <Qt3DRender/QBuffer>
#include <Qt3DRender/QAttribute>
#include <Qt3DRender/qbufferdatagenerator.h>
#include <QHash>
#include <iomanip>
#include <upns/layertypes/assettype.h>
#include <QSharedPointer>

class QmlPlyMeshGeometryPrivate
{
public:
    QmlPlyMeshGeometryPrivate()
        :m_vertexBuffer( nullptr ),
         m_asset( nullptr ){}
    Qt3DRender::QBuffer *m_vertexBuffer;
    Qt3DRender::QBuffer *m_indexBuffer;
    AssetPtr m_asset;
};

QmlPlyMeshGeometry::QmlPlyMeshGeometry(Qt3DCore::QNode *parent)
    :m_p(new QmlPlyMeshGeometryPrivate)
{
    m_p->m_vertexBuffer = new Qt3DRender::QBuffer(Qt3DRender::QBuffer::VertexBuffer, this);
    m_p->m_indexBuffer = new Qt3DRender::QBuffer(Qt3DRender::QBuffer::IndexBuffer, this);
}

QmlPlyMeshGeometry::~QmlPlyMeshGeometry()
{
    delete m_p;
}

void QmlPlyMeshGeometry::updateVertices()
{
    //QMetaObject::invokeMethod(this, "updateAttributes", Qt::QueuedConnection);

    std::vector<float> verts;
    std::vector<uint32_t> faces;
//    std::vector<uint32_t> faces16;
    uint32_t vertexCount = m_p->m_asset->first.request_properties_from_element("vertex", { "x", "y", "z" }, verts);
    m_p->m_asset->first.request_properties_from_element("face", { "vertex_indices" }, faces, 3);
    m_p->m_asset->first.read(*m_p->m_asset->second);
//    faces16.reserve(faces.size());
//    for(uint32_t &t : faces)
//    {
//        if(t >= vertexCount) {
//            qDebug() << "ERROR DBG, IDX was: " << t;
//        }
//        faces16.push_back(t);
//    }
    m_p->m_vertexBuffer->setData(QByteArray(reinterpret_cast<char*>(verts.data()), verts.size()*sizeof(float)));
    m_p->m_indexBuffer->setData(QByteArray(reinterpret_cast<char*>(faces.data()), faces.size()*sizeof(uint32_t)));
    updateAttributes(vertexCount);
}

void QmlPlyMeshGeometry::updateAttributes(uint32_t vertexCount)
{
    // completely rebuild attribute list and remove all previous attributes
    QVector<Qt3DRender::QAttribute *> atts = attributes();
    Q_FOREACH(Qt3DRender::QAttribute *attr, atts)
    {
//        if(attr->attributeType() == Qt3DRender::QAttribute::VertexAttribute)
//        {
            removeAttribute(attr);
            attr->deleteLater();
//        }
//        else
//        {
//            qDebug() << "skipped index";
//        }
    }
    Qt3DRender::QAttribute* attrib = new Qt3DRender::QAttribute(this);
    attrib->setName(Qt3DRender::QAttribute::defaultPositionAttributeName());
    attrib->setDataType(Qt3DRender::QAttribute::Float);
    attrib->setDataSize(3);
    attrib->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
    attrib->setBuffer(m_p->m_vertexBuffer);
    attrib->setByteStride(3*sizeof(float));
    attrib->setByteOffset(0);
    attrib->setCount(vertexCount);

    Qt3DRender::QAttribute* idxAttrib = new Qt3DRender::QAttribute(this);
    idxAttrib->setAttributeType(Qt3DRender::QAttribute::IndexAttribute);
    idxAttrib->setVertexBaseType(Qt3DRender::QAttribute::UnsignedInt);
    idxAttrib->setBuffer(m_p->m_indexBuffer);
    uint faceCount = m_p->m_indexBuffer->data().size()/sizeof(uint32_t);
    idxAttrib->setCount(faceCount);
    addAttribute(idxAttrib);
    setBoundingVolumePositionAttribute(attrib);
}

void QmlPlyMeshGeometry::setAsset(AssetPtr asset)
{
    if (m_p->m_asset == asset)
        return;

    m_p->m_asset = asset;
    updateVertices();
    Q_EMIT assetChanged(asset);
}
