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
    Eigen::Quaternionf q(
          -rotation.x() * v.x() - rotation.y() * v.y() - rotation.z() * v.z()
        , rotation.w() * v.x() + rotation.y() * v.z() - rotation.z() * v.y()
        , rotation.w() * v.y() + rotation.z() * v.x() - rotation.x() * v.z()
        , rotation.w() * v.z() + rotation.x() * v.y() - rotation.y() * v.x()
        );
    q *= rotation.inverse();
    return Eigen::Vector3f(q.x(),q.y(),q.z());
}
