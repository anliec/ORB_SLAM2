#ifndef KABSCH_H
#define KABSCH_H

#include <Eigen/Geometry>

Eigen::Affine3d Find3DAffineTransform(Eigen::Matrix3Xd in, Eigen::Matrix3Xd out);

#endif // KABSCH_H
