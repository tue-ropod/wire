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

#include "wire_state_estimators/PositionAndDimensionFilter.h"

PositionAndDimensionFilter::PositionAndDimensionFilter() : 

t_last_update_(0),
t_last_propagation_(0),
properties_(0)
{
}

PositionAndDimensionFilter::PositionAndDimensionFilter(const PositionAndDimensionFilter& orig) : mhf::IStateEstimator(orig), 
t_last_update_(orig.t_last_update_),
t_last_propagation_(orig.t_last_propagation_),
properties_(0)

{
    if (orig.properties_) 
    {
        properties_ = new tracking::FeatureProperties(*orig.properties_);
    }
}

PositionAndDimensionFilter::~PositionAndDimensionFilter() {
        delete properties_;
        
}

PositionAndDimensionFilter* PositionAndDimensionFilter::clone() const {
    return new PositionAndDimensionFilter(*this);
}

void PositionAndDimensionFilter::propagate(const mhf::Time& time) {

    if (t_last_propagation_ == 0) {
        t_last_propagation_ = time;
        return;
    }

    mhf::Duration dt = time - t_last_propagation_;
    t_last_propagation_ = time;

    assert(dt >= 0);

    if (!properties_) {
        return;
    }

    float Q = 0.4;
    pbl::Vector diagQRect, diagQCircle;
    diagQRect << Q << Q << Q << 20*Q << 20*Q << 20*Q << Q << Q << arma::endr; // TODO why this difference in Q?
    diagQCircle << Q << Q << Q << Q << Q << arma::endr;
    pbl::Matrix QmRectangle = arma::diagmat( diagQRect );
    pbl::Matrix QmCircle = arma::diagmat( diagQCircle );
   
    properties_->propagateRectangleFeatures( QmRectangle, dt );
    properties_->propagateCircleFeatures( QmCircle, dt);
}

void PositionAndDimensionFilter::update(std::shared_ptr<const pbl::PDF> z, const mhf::Time& time) {
    if (z->type() == pbl::PDF::HYBRID) {
        std::shared_ptr<const pbl::Hybrid> H = pbl::PDFtoHybrid(z);
        
        if (!properties_) 
        {
            properties_ = new tracking::FeatureProperties();
            properties_->setMeasuredFeatureProperties(H);
        }
       else 
       {     
            tracking::FeatureProperties measuredProperties; 
            measuredProperties.setMeasuredFeatureProperties(H);
            
            pbl::Vector z_kRectangle = measuredProperties.rectangle_.get_H()*measuredProperties.rectangle_.getState();
            pbl::Vector z_kCircle = measuredProperties.circle_.get_H()*measuredProperties.circle_.getState();
            
            properties_->updateRectangleFeatures( measuredProperties.rectangle_.getCovariance(), z_kRectangle );
            properties_->updateCircleFeatures( measuredProperties.circle_.getCovariance(), z_kCircle );
            properties_->updateProbabilities(measuredProperties.featureProbabilities_);
        }
        
    } else {
        printf("PositionAndDimensionFilter can only be updated with Hybrids.\n");
    }
}

void PositionAndDimensionFilter::reset() {
    delete properties_;
    properties_ = 0;
}

std::shared_ptr<const pbl::PDF> PositionAndDimensionFilter::getValue() const {  
   if(properties_)
   {    
           return properties_->getPDFSmall();
   }
   
    std::cout << "PositionAndDimensionFilter::getValue(): SOMETHINGS WRONG" << std::endl;
}

std::shared_ptr<const pbl::PDF> PositionAndDimensionFilter::getFullValue() const {  
   if(properties_)
   {
           return properties_->getPDF();
   }
   
    std::cout << "PositionAndDimensionFilter::getValue(): SOMETHINGS WRONG" << std::endl;
}

bool PositionAndDimensionFilter::setParameter(const std::string& param, bool b) {
    return false;
}

bool PositionAndDimensionFilter::setParameter(const std::string &param, double v) {
    return true;
}

bool PositionAndDimensionFilter::setParameter(const std::string& param, const std::string& s)
{
    return false;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( PositionAndDimensionFilter, mhf::IStateEstimator )