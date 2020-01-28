/************************************************************************
 *  Copyright (C) 2012 Eindhoven University of Technology (TU/e).       *
 *  All rights reserved.                                                *
 ************************************************************************
 *  Redistribution and use in source and binary forms, with or without  *
 *  modification, are permitted provided that the following conditions  *
 *  are met:                                                            *
 *                                                                      *
 *      1.  Redistributions of source code must retain the above        *
 *          copyright notice, this list of conditions and the following *
 *          disclaimer.                                                 *
 *                                                                      *
 *      2.  Redistributions in binary form must reproduce the above     *
 *          copyright notice, this list of conditions and the following *
 *          disclaimer in the documentation and/or other materials      *
 *          provided with the distribution.                             *
 *                                                                      *
 *  THIS SOFTWARE IS PROVIDED BY TU/e "AS IS" AND ANY EXPRESS OR        *
 *  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED      *
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE  *
 *  ARE DISCLAIMED. IN NO EVENT SHALL TU/e OR CONTRIBUTORS BE LIABLE    *
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR        *
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT   *
 *  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;     *
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF       *
 *  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT           *
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE   *
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH    *
 *  DAMAGE.                                                             *
 *                                                                      *
 *  The views and conclusions contained in the software and             *
 *  documentation are those of the authors and should not be            *
 *  interpreted as representing official policies, either expressed or  *
 *  implied, of TU/e.                                                   *
 ************************************************************************/

#include "wire_state_estimators/PositionFilter.h"
#include "wire_state_estimators/KalmanFilter.h"

PositionFilter::PositionFilter() : t_last_update_(0), t_last_propagation_(0), kalman_filter_(0),
    fixed_pdf_(0), max_acceleration_(0), fixed_pdf_cov_(0), kalman_timeout_(0), ProcessModel_(ProcessModel::NONE) {
}

PositionFilter::PositionFilter(const PositionFilter& orig) : mhf::IStateEstimator(orig), t_last_update_(orig.t_last_update_),
    t_last_propagation_(orig.t_last_propagation_), kalman_filter_(0), fixed_pdf_(0), max_acceleration_(orig.max_acceleration_),
    fixed_pdf_cov_(orig.fixed_pdf_cov_ ), kalman_timeout_(orig.kalman_timeout_), ProcessModel_(orig.ProcessModel_) {

    if (orig.fixed_pdf_) {
        fixed_pdf_ = orig.fixed_pdf_->CloneMethod();
    }

    if (orig.kalman_filter_) {
        kalman_filter_ = new KalmanFilter(*orig.kalman_filter_);
    }
}

PositionFilter::~PositionFilter() {
    delete kalman_filter_;
    //delete fixed_pdf_;
}

/*PositionFilter* PositionFilter::clone() const {
    return new PositionFilter(*this);
}*/

void PositionFilter::propagate(const mhf::Time& time) {

    if (t_last_propagation_ == 0) {
        t_last_propagation_ = time;
        return;
    }

    mhf::Duration dt = time - t_last_propagation_;
    t_last_propagation_ = time;

    assert(dt >= 0);

    if (!kalman_filter_) {
        return;
    }

    if ((time - t_last_update_) > kalman_timeout_ && kalman_timeout_ > 0) {
        if (!fixed_pdf_) {
            int dimensions = kalman_filter_->getGaussian()->getMean().n_rows;
            pbl::Matrix cov = arma::eye(dimensions, dimensions) * fixed_pdf_cov_;
             fixed_pdf_ = std::make_shared<pbl::Gaussian>(kalman_filter_->getGaussian()->getMean(), cov);
        } else {
            fixed_pdf_->setMean(kalman_filter_->getGaussian()->getMean());
        }

        delete kalman_filter_;
        kalman_filter_ = 0;
        return;
    }

    // TODO: fix the kalman filter update (we shouldn't need a loop here...)
    //mhf::Duration small_dt = 0.05;
    mhf::Duration small_dt = kalman_timeout_;
    if (dt < small_dt) {
        kalman_filter_->propagate(dt);
    } else {
        double total_dt = 0;
        for(; total_dt < dt; total_dt += small_dt) {
            kalman_filter_->propagate(small_dt);
        }
        if (total_dt < dt) {
            kalman_filter_->propagate(dt - total_dt);
        }
    }
}

void PositionFilter::update(std::shared_ptr<const pbl::PDF> z, const mhf::Time& time) {
    t_last_update_ = time;

    if (z->type() == pbl::PDF::GAUSSIAN) {
        std::shared_ptr<const pbl::Gaussian> G = pbl::PDFtoGaussian(z);

        if (!kalman_filter_) {
            kalman_filter_ = new KalmanFilter(z->dimensions(), ProcessModel_);
            kalman_filter_->setMaxAcceleration(max_acceleration_);
            kalman_filter_->init(G);
        } else {
            kalman_filter_->update(G);
        }
    } else {
        printf("PositionFilter can only be updated with Gaussians.\n");
    }
}

void PositionFilter::reset() {
    delete kalman_filter_;
    kalman_filter_ = 0;

    //delete fixed_pdf_;
    fixed_pdf_ = 0;
}

std::shared_ptr<const pbl::PDF> PositionFilter::getValue() const {
    if (kalman_filter_) {
        return kalman_filter_->getGaussian();
    } else if (fixed_pdf_) {
        return fixed_pdf_;
    }

    std::cout << "SOMETHINGS WRONG" << std::endl;
    //return std::shared_ptr<const pbl::PDF>(nullptr); // Temp to print out the properties
}

std::shared_ptr<const pbl::PDF> PositionFilter::getFullValue() const {
    return this->getValue();
}

bool PositionFilter::setParameter(const std::string& param, bool b) {
    return false;
}

bool PositionFilter::setParameter(const std::string &param, double v) {
    if (param == "max_acceleration") {
        max_acceleration_ = v;
        if (kalman_filter_) {
            kalman_filter_->setMaxAcceleration(max_acceleration_);
        }
    } else if (param == "fixed_pdf_cov") {
        fixed_pdf_cov_ = v;
    } else if (param == "kalman_timeout") {
        kalman_timeout_ = v;
    } else {
        return false;
    }
    return true;
}

bool PositionFilter::setParameter(const std::string& param, const std::string& s)
{
        if (param == "process_model") 
        {
                if(s == "constant")
                {
                        this->ProcessModel_ = ProcessModel::CONSTANT;
                        return true;
                } else if ( s == "constant_velocity")
                {
                        this->ProcessModel_ = ProcessModel::CONSTANT_VELOCITY;
                        return true;
                }
        }
         return false;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( PositionFilter, mhf::IStateEstimator )
