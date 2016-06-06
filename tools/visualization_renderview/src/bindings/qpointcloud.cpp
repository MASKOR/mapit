#include "qpointcloud.h"

//QPointcloud::QPointcloud()
//    :m_pointcloud(NULL)
//{
//}

QPointcloud::QPointcloud(pcl::PCLPointCloud2Ptr pointcloud)
    :m_pointcloud(pointcloud)
{
    for(int i=0 ; i < m_pointcloud->fields.size() ; ++i)
    {
        m_fields.append( new QPointfield(this, &m_pointcloud->fields[i]) );
    }
}

quint32 QPointcloud::height() const
{
    return m_pointcloud->height;
}

quint32 QPointcloud::width() const
{
    return  m_pointcloud->width;
}

QQmlListProperty<QPointfield> QPointcloud::fields()
{
    return QQmlListProperty<QPointfield>(this, static_cast<void*>(m_pointcloud.get()), &fields_append, &fields_count, &fields_at, &fields_clear);
}

void QPointcloud::fields_append(QQmlListProperty<QPointfield> *self, QPointfield *f)
{
    Q_ASSERT_X(false, "QPointcloud::fields_append", "Must not be called. ");
    pcl::PCLPointCloud2 *p = static_cast<pcl::PCLPointCloud2*>(self->data);
    p->fields.push_back(*f->getPointfield());
}

int QPointcloud::fields_count(QQmlListProperty<QPointfield> *self)
{
    pcl::PCLPointCloud2 *p = static_cast<pcl::PCLPointCloud2*>(self->data);
    return p->fields.size();
}

QPointfield *QPointcloud::fields_at(QQmlListProperty<QPointfield> *self, int i)
{
    pcl::PCLPointCloud2 *p = static_cast<pcl::PCLPointCloud2*>(self->data);
    return new QPointfield(&p->fields.at(i));
}

void QPointcloud::fields_clear(QQmlListProperty<QPointfield> *self)
{
    pcl::PCLPointCloud2 *p = static_cast<pcl::PCLPointCloud2*>(self->data);
    p->fields.clear();
}

quint8 QPointcloud::is_bigendian() const
{
    return m_pointcloud->is_bigendian;
}

quint32 QPointcloud::point_step() const
{
    return m_pointcloud->point_step;
}

quint32 QPointcloud::row_step() const
{
    return m_pointcloud->row_step;
}

QByteArray QPointcloud::data() const
{
    return QByteArray::fromRawData(reinterpret_cast<const char*>(&m_pointcloud->data[0]), m_pointcloud->data.size());
}

quint8 QPointcloud::is_dense() const
{
    return m_pointcloud->is_dense;
}

void QPointcloud::setHeight(quint32 height)
{
    if (m_pointcloud->height == height)
        return;

    m_pointcloud->height = height;
    Q_EMIT heightChanged(height);
}

void QPointcloud::setWidth(quint32 width)
{
    if (m_pointcloud->width == width)
        return;

    m_pointcloud->width = width;
    Q_EMIT widthChanged(width);
}

void QPointcloud::setIs_bigendian(quint8 is_bigendian)
{
    if (m_pointcloud->is_bigendian == is_bigendian)
        return;

    m_pointcloud->is_bigendian = is_bigendian;
    Q_EMIT is_bigendianChanged(is_bigendian);
}

void QPointcloud::setPoint_step(quint32 point_step)
{
    if (m_pointcloud->point_step == point_step)
        return;

    m_pointcloud->point_step = point_step;
    Q_EMIT point_stepChanged(point_step);
}

void QPointcloud::setRow_step(quint32 row_step)
{
    if (m_pointcloud->row_step == row_step)
        return;

    m_pointcloud->row_step = row_step;
    Q_EMIT row_stepChanged(row_step);
}

void QPointcloud::setData(QByteArray data)
{
    m_pointcloud->data.resize(data.size());
    memcpy(&m_pointcloud->data[0], data.data(), m_pointcloud->data.size());
    Q_EMIT dataChanged(data);
}
