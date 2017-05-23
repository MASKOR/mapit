#include "upns/ui/bindings/qmlplymeshgeometry.h"
#include <Qt3DRender/QBuffer>
#include <Qt3DRender/QAttribute>
#include <Qt3DRender/qbufferdatagenerator.h>
#include <QHash>
#include <iomanip>
#include <upns/layertypes/assettype.h>
#include <QSharedPointer>

template<typename T>
class PlyMeshVertexDataGenerator : public Qt3DRender::QBufferDataGenerator
{
public:
    PlyMeshVertexDataGenerator(upnsAssetPtr asset, const std::string & elementName, std::vector<std::string> propertyKeys)
        : m_asset(asset)
        , m_name(elementName)
        , m_propertyKeys(propertyKeys)
    {
    }

    QByteArray operator ()() Q_DECL_OVERRIDE
    {
        std::vector<T> verts;
        m_asset->request_properties_from_element(m_name, m_propertyKeys, verts);
        return QByteArray(reinterpret_cast<char*>(verts.data()), verts.size()*sizeof(float));
    }

    bool operator ==(const Qt3DRender::QBufferDataGenerator &other) const Q_DECL_OVERRIDE
    {
        const PlyMeshVertexDataGenerator *otherFunctor = Qt3DRender::functor_cast<PlyMeshVertexDataGenerator>(&other);
        if (otherFunctor != NULL)
            return otherFunctor->m_asset == m_asset
                && otherFunctor->m_name == m_name
                && otherFunctor->m_propertyKeys == m_propertyKeys;
        return false;
    }


private:
    upnsAssetPtr m_asset;
    std::string m_name;
    std::vector<std::string> m_propertyKeys;
};
class PlyMeshVertexDataGeneratorFloat : public PlyMeshVertexDataGenerator<float>
{
public:
    PlyMeshVertexDataGeneratorFloat(upnsAssetPtr asset, const std::string & elementName, std::vector<std::string> propertyKeys)
        :PlyMeshVertexDataGenerator(asset, elementName, propertyKeys) {}
    QT3D_FUNCTOR(PlyMeshVertexDataGeneratorFloat)
};
class PlyMeshVertexDataGeneratorUInt : public PlyMeshVertexDataGenerator<float>
{
public:
    PlyMeshVertexDataGeneratorUInt(upnsAssetPtr asset, const std::string & elementName, std::vector<std::string> propertyKeys)
        :PlyMeshVertexDataGenerator(asset, elementName, propertyKeys) {}
    QT3D_FUNCTOR(PlyMeshVertexDataGeneratorUInt)
};

class QmlPlyMeshGeometryPrivate
{
public:
    QmlPlyMeshGeometryPrivate()
        :m_vertexBuffer( nullptr ),
         m_asset( nullptr ){}
    Qt3DRender::QBuffer *m_vertexBuffer;
    Qt3DRender::QBuffer *m_indexBuffer;
    upnsAssetPtr m_asset;
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
    updateAttributes();
    //QMetaObject::invokeMethod(this, "updateAttributes", Qt::QueuedConnection);
    m_p->m_vertexBuffer->setDataGenerator(Qt3DRender::QBufferDataGeneratorPtr(new PlyMeshVertexDataGeneratorFloat(m_p->m_asset, "vertex", { "x", "y", "z" })));
    m_p->m_indexBuffer->setDataGenerator(Qt3DRender::QBufferDataGeneratorPtr(new PlyMeshVertexDataGeneratorUInt(m_p->m_asset, "face", { "vertex_indices" })));
}

void QmlPlyMeshGeometry::updateAttributes()
{
    // completely rebuild attribute list and remove all previous attributes
    QVector<Qt3DRender::QAttribute *> atts = attributes();
    Q_FOREACH(Qt3DRender::QAttribute *attr, atts)
    {
        if(attr->attributeType() == Qt3DRender::QAttribute::VertexAttribute)
        {
            removeAttribute(attr);
            attr->deleteLater();
        }
        else
        {
            qDebug() << "skipped index";
        }
    }
    std::vector<float> verts;
    uint32_t vertexCount;
    vertexCount = m_p->m_asset->request_properties_from_element("vertex", { "x", "y", "z" }, verts);
    Qt3DRender::QAttribute* attrib = new Qt3DRender::QAttribute(this);
    attrib->setName(Qt3DRender::QAttribute::defaultPositionAttributeName());
    attrib->setDataType(Qt3DRender::QAttribute::Float);
    attrib->setDataSize(3);
    attrib->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
    attrib->setBuffer(m_p->m_vertexBuffer);
    attrib->setByteStride(3*sizeof(float));
    attrib->setByteOffset(0);
    attrib->setCount(vertexCount);
    addAttribute(attrib);
    setBoundingVolumePositionAttribute(attrib);
}

void QmlPlyMeshGeometry::setAsset(upnsAssetPtr asset)
{
    if (m_p->m_asset == asset)
        return;

    m_p->m_asset = asset;
    updateVertices();
    Q_EMIT assetChanged(asset);
}
