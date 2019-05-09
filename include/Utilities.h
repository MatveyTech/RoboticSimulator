#pragma once

#include <Eigen/Sparse>

using namespace Eigen;

#define COLOR(r,g,b) RowVector3d(r / 255., g / 255., b / 225.)

MatrixXd TransformP(Eigen::MatrixXd &V, Eigen::Matrix4d &tr);
