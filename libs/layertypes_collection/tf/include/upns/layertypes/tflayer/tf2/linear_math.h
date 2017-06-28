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
  Eigen::Vector3f q_v = rotation * v;
  Eigen::Quaternionf q(q_v.x(), q_v.y(), q_v.z(), 0);
  q *= rotation.inverse();
  return Eigen::Vector3f(q.x(),q.y(),q.z());
}
