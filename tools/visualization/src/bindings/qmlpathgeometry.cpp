#include "upns/ui/bindings/qmlpathgeometry.h"
#include <Qt3DRender/QBuffer>
#include <Qt3DRender/QAttribute>
#include <Qt3DRender/qbufferdatagenerator.h>
#include <QHash>
#include <iomanip>
#include <upns/layertypes/pose_path.h>
#include <QSharedPointer>

class PathVertexDataGenerator : public Qt3DRender::QBufferDataGenerator
{
public:
    PathVertexDataGenerator(TfMatPosePathPtr path)
        : m_path(path)
    {
    }

    QByteArray operator ()() Q_DECL_OVERRIDE
    {
        QByteArray arr;
        arr.resize(m_path->size()*sizeof(float)*3);
        int i=0;
        for(TfMatPosePath::const_iterator iter = m_path->cbegin() ; iter!=m_path->cend() ; iter++)
        {
            float* val = reinterpret_cast<float*>(&arr.data()[i]);
            val[0] = static_cast<float>(iter->data()[12]);
            val[1] = static_cast<float>(iter->data()[13]);
            val[2] = static_cast<float>(iter->data()[14]);
            i+=sizeof(float)*3;
        }
        return arr;
    }

    bool operator ==(const Qt3DRender::QBufferDataGenerator &other) const Q_DECL_OVERRIDE
    {
        const PathVertexDataGenerator *otherFunctor = Qt3DRender::functor_cast<PathVertexDataGenerator>(&other);
        if (otherFunctor != NULL)
            return otherFunctor->m_path == m_path;
        return false;
    }

    QT3D_FUNCTOR(PathVertexDataGenerator)

private:
    TfMatPosePathPtr m_path;
};

class QmlPathGeometryPrivate
{
public:
    QmlPathGeometryPrivate()
        :m_vertexBuffer( nullptr ),
         m_path( nullptr ){}
    Qt3DRender::QBuffer *m_vertexBuffer;
    TfMatPosePathPtr m_path;
};

QmlPathGeometry::QmlPathGeometry(Qt3DCore::QNode *parent)
    :m_p(new QmlPathGeometryPrivate)
{
    m_p->m_vertexBuffer = new Qt3DRender::QBuffer(Qt3DRender::QBuffer::VertexBuffer, this);
}

QmlPathGeometry::~QmlPathGeometry()
{
    delete m_p;
}

void QmlPathGeometry::updateVertices()
{
    updateAttributes();
    //QMetaObject::invokeMethod(this, "updateAttributes", Qt::QueuedConnection);
    m_p->m_vertexBuffer->setDataGenerator(Qt3DRender::QBufferDataGeneratorPtr(new PathVertexDataGenerator(m_p->m_path)));
}

void QmlPathGeometry::updateAttributes()
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
    Qt3DRender::QAttribute* attrib = new Qt3DRender::QAttribute(this);
    attrib->setName(Qt3DRender::QAttribute::defaultPositionAttributeName());
    attrib->setDataType(Qt3DRender::QAttribute::Float);
    attrib->setDataSize(3);
    attrib->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
    attrib->setBuffer(m_p->m_vertexBuffer);
    attrib->setByteStride(3*sizeof(float));
    attrib->setByteOffset(0);
    attrib->setCount(m_p->m_path->size());
    addAttribute(attrib);
    setBoundingVolumePositionAttribute(attrib);
}

void QmlPathGeometry::setPath(TfMatPosePathPtr path)
{
    if (m_p->m_path == path)
        return;

    m_p->m_path = path;
    updateVertices();
    Q_EMIT pathChanged(path);
}
