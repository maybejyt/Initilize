#pragma once
#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "utility.h"
#include "tic_toc.h"
#include "estimator.h"
#include "feature_manager.h"
#include "parameters.h"
#include "sophus/so3.h"
#include "sophus/se3.h"
class PlaneFactor : public ceres::SizedCostFunction<3, 6, 2, 1>
{
  public:
    PlaneFactor(IntegrationBase* _pre_integration,PlaneParams& planeparams);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);
    IntegrationBase* pre_integration;
    PlaneParams planeparams_;
    Matrix3d crossproMatrix(Eigen::Vector3d a) const;
   
};
