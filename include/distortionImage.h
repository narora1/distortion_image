#ifndef DISTORTIONIMAGE_H
#define DISTORTIONIMAGE_H

#include <iostream>
#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::DynamicAutoDiffCostFunction;

struct DistortionImageFunctor {
    typedef DynamicAutoDiffCostFunction <DistortionImageFunctor, 10> DistortionImageCostFunction;

    DistortionImageFunctor(std::vector<double> measurement_)
        :   measurement(measurement_) {}

    template <typename T>
    bool operator() (T const* const* x, T* residuals) const
    {
        T m(0);
        m = *x[0];
        if (!std::isfinite(measurement.at(5)))
            residuals[0] = T(0);
        else
            residuals[0] = T(sqrt(/* (measurement.at(3) - measurement.at(0) ) * (measurement.at(3) - measurement.at(0) ) + (measurement.at(4) - measurement.at(1) ) * \
                               (measurement.at(4) - measurement.at(1) ) +*/ (m * measurement.at(5) - measurement.at(2) ) * (m * measurement.at(5) - measurement.at(2) )) );

        return true;
    }

    static DistortionImageCostFunction* Create(std::vector <double> measurement, int size)
    {
        DistortionImageFunctor* constraint = new DistortionImageFunctor(measurement);
        DistortionImageCostFunction* cost_function = new DistortionImageCostFunction(constraint);
        cost_function->AddParameterBlock(1);
        cost_function->SetNumResiduals(1);
        return (cost_function);
    }

    const std::vector<double> measurement;
};


#endif // DISTORTIONIMAGE_H
