// Local integration patch, GPL-3.0, like the surrounding VINS-Fusion code.
#pragma once
#include <ceres/ceres.h>

// A single prior on the initial sensor bias, subsequently propagated by
// VINS' ordinary marginalization and IMU random walk. Static stereo cannot
// distinguish absolute gravity tilt from accelerometer bias. The sigmas are
// the original Iris IMU's turn-on bias distributions, not ground truth.
class ImuBiasPrior : public ceres::SizedCostFunction<6, 9>
{
public:
    ImuBiasPrior(double acceleration_sigma, double gyro_sigma)
        : acceleration_weight_(1.0 / acceleration_sigma), gyro_weight_(1.0 / gyro_sigma) {}

    bool Evaluate(double const* const* parameters, double* residuals,
                  double** jacobians) const override
    {
        for (int i = 0; i < 6; ++i)
        {
            const double weight = i < 3 ? acceleration_weight_ : gyro_weight_;
            residuals[i] = parameters[0][i + 3] * weight;
            if (jacobians && jacobians[0])
            {
                for (int j = 0; j < 9; ++j)
                    jacobians[0][i * 9 + j] = j == i + 3 ? weight : 0.0;
            }
        }
        return true;
    }

private:
    double acceleration_weight_, gyro_weight_;
};
