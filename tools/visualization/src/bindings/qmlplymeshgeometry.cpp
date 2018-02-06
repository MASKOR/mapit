#include "upns/ui/bindings/qmlplymeshgeometry.h"
#include <Qt3DRender/QBuffer>
#include <Qt3DRender/QAttribute>
#include <Qt3DRender/qbufferdatagenerator.h>
#include <QHash>
#include <iomanip>
#include <upns/layertypes/assettype.h>
#include <QSharedPointer>

class PlyVertexDataGenerator : public Qt3DRender::QBufferDataGenerator
{
public:
    PlyVertexDataGenerator(std::vector<float> &verts)
        : m_verts(verts)
    {}
    QByteArray operator ()() Q_DECL_OVERRIDE
    {
        QByteArray arr(reinterpret_cast<char*>(m_verts.data()), m_verts.size()*sizeof(float));
        return arr;
    }

    bool operator ==(const Qt3DRender::QBufferDataGenerator &other) const Q_DECL_OVERRIDE
    {
        const PlyVertexDataGenerator *otherFunctor = Qt3DRender::functor_cast<PlyVertexDataGenerator>(&other);
        if (otherFunctor != NULL)
            return otherFunctor->m_verts == m_verts;
        return false;
    }
    QT3D_FUNCTOR(PlyVertexDataGenerator)

private:
    std::vector<float> m_verts;
};

class PlyIndexDataGenerator : public Qt3DRender::QBufferDataGenerator
{
public:
    PlyIndexDataGenerator(std::vector<uint32_t> &index)
        : m_index(index)
    {}
    QByteArray operator ()() Q_DECL_OVERRIDE
    {
        QByteArray arr(reinterpret_cast<char*>(m_index.data()), m_index.size()*sizeof(uint32_t));
        return arr;
    }

    bool operator ==(const Qt3DRender::QBufferDataGenerator &other) const Q_DECL_OVERRIDE
    {
        const PlyIndexDataGenerator *otherFunctor = Qt3DRender::functor_cast<PlyIndexDataGenerator>(&other);
        if (otherFunctor != NULL)
            return otherFunctor->m_index == m_index;
        return false;
    }
    QT3D_FUNCTOR(PlyIndexDataGenerator)

private:
    std::vector<uint32_t> m_index;
};

class QmlPlyMeshGeometryPrivate
{
public:
    QmlPlyMeshGeometryPrivate()
        :m_vertexBuffer( nullptr ),
         m_normalsBuffer( nullptr ),
         m_asset( nullptr ){}
    Qt3DRender::QBuffer *m_vertexBuffer;
    Qt3DRender::QBuffer *m_normalsBuffer;
    Qt3DRender::QBuffer *m_indexBuffer;
    AssetPtr m_asset;
};

QmlPlyMeshGeometry::QmlPlyMeshGeometry(Qt3DCore::QNode *parent)
    :m_p(new QmlPlyMeshGeometryPrivate)
{
    m_p->m_vertexBuffer = new Qt3DRender::QBuffer(Qt3DRender::QBuffer::VertexBuffer, this);
    m_p->m_normalsBuffer = new Qt3DRender::QBuffer(Qt3DRender::QBuffer::VertexBuffer, this);
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
    std::vector<float> normals;
    std::vector<uint32_t> faces;
////    std::vector<uint32_t> faces16;
    uint32_t vertexCount = m_p->m_asset->first.request_properties_from_element("vertex", { "x", "y", "z" }, verts);
    uint32_t normalsCount = m_p->m_asset->first.request_properties_from_element("vertex", { "nx", "ny", "nz" }, normals);
    m_p->m_asset->first.request_properties_from_element("face", { "vertex_indices" }, faces, 3);
    m_p->m_asset->first.read(*m_p->m_asset->second);
////    faces16.reserve(faces.size());
////    for(uint32_t &t : faces)
////    {
////        if(t >= vertexCount) {
////            qDebug() << "ERROR DBG, IDX was: " << t;
////        }
////        faces16.push_back(t);
////    }

//    verts.push_back(-1.0); verts.push_back(-1.0); verts.push_back(-1.0);
//    verts.push_back(1.0); verts.push_back(-1.0); verts.push_back(-1.0);
//    verts.push_back(1.0); verts.push_back(1.0); verts.push_back(-1.0);
//    verts.push_back(10.0); verts.push_back(1.0); verts.push_back(1.0);
//    verts.push_back(1.0); verts.push_back(10.0); verts.push_back(1.0);
//    verts.push_back(1.0); verts.push_back(1.0); verts.push_back(10.0);
//    faces.push_back(0); faces.push_back(1); faces.push_back(2);
//    faces.push_back(1); faces.push_back(2); faces.push_back(3);
//    faces.push_back(2); faces.push_back(0); faces.push_back(3);
//    faces.push_back(3); faces.push_back(0); faces.push_back(2);
//    faces.push_back(3); faces.push_back(4); faces.push_back(5);
//    faces.push_back(3); faces.push_back(5); faces.push_back(4);
//    uint32_t vertexCount = 6;

//    m_p->m_vertexBuffer->setDataGenerator(Qt3DRender::QBufferDataGeneratorPtr(new PlyVertexDataGenerator(verts)));
//    m_p->m_indexBuffer->setDataGenerator(Qt3DRender::QBufferDataGeneratorPtr(new PlyIndexDataGenerator(faces)));
    m_p->m_vertexBuffer->setData(QByteArray(reinterpret_cast<char*>(verts.data()), verts.size()*sizeof(float)));
    m_p->m_normalsBuffer->setData(QByteArray(reinterpret_cast<char*>(normals.data()), normals.size()*sizeof(float)));
    m_p->m_indexBuffer->setData(QByteArray(reinterpret_cast<char*>(faces.data()), faces.size()*sizeof(uint32_t)));
    updateAttributes(vertexCount, normalsCount >= 0);
}

void QmlPlyMeshGeometry::updateAttributes(uint32_t vertexCount, bool hasNormals)
{
    // completely rebuild attribute list and remove all previous attributes
    QVector<Qt3DRender::QAttribute *> atts = attributes();
    Q_FOREACH(Qt3DRender::QAttribute *attr, atts)
    {
            removeAttribute(attr);
            attr->deleteLater();
    }
    Qt3DRender::QAttribute* attrib = new Qt3DRender::QAttribute(nullptr);
    attrib->setName(Qt3DRender::QAttribute::defaultPositionAttributeName());
    attrib->setDataType(Qt3DRender::QAttribute::Float);
    attrib->setDataSize(3);
    attrib->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
    attrib->setBuffer(m_p->m_vertexBuffer);
    attrib->setByteStride(3*sizeof(float));
    attrib->setByteOffset(0);
    attrib->setCount(vertexCount);
    setBoundingVolumePositionAttribute(attrib);
    addAttribute(attrib);

    if(hasNormals)
    {
        Qt3DRender::QAttribute* attrib = new Qt3DRender::QAttribute(nullptr);
        attrib->setName(Qt3DRender::QAttribute::defaultNormalAttributeName());
        attrib->setDataType(Qt3DRender::QAttribute::Float);
        attrib->setDataSize(3);
        attrib->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
        attrib->setBuffer(m_p->m_normalsBuffer);
        attrib->setByteStride(3*sizeof(float));
        attrib->setByteOffset(0);
        attrib->setCount(vertexCount);
        setBoundingVolumePositionAttribute(attrib);
        addAttribute(attrib);
    }

    Qt3DRender::QAttribute* idxAttrib = new Qt3DRender::QAttribute(nullptr);
    idxAttrib->setAttributeType(Qt3DRender::QAttribute::IndexAttribute);
    idxAttrib->setVertexBaseType(Qt3DRender::QAttribute::UnsignedInt);
    idxAttrib->setBuffer(m_p->m_indexBuffer);
    uint indexCount = m_p->m_indexBuffer->data().size()/sizeof(uint32_t);
    idxAttrib->setCount(indexCount);
    addAttribute(idxAttrib);
}

void QmlPlyMeshGeometry::setAsset(AssetPtr asset)
{
    if (m_p->m_asset == asset)
        return;

    m_p->m_asset = asset;
    updateVertices();
    Q_EMIT assetChanged(asset);
}
