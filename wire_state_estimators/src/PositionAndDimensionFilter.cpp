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
//#include "KalmanFilter.h"

PositionAndDimensionFilter::PositionAndDimensionFilter() : 

t_last_update_(0),
t_last_propagation_(0),
properties_(0)
//kalman_filter_(0),
//fixed_pdf_(0),
//max_acceleration_(0),
//fixed_pdf_cov_(0),
//kalman_timeout_(0),
//ProcessModel_(ProcessModel::NONE)
{
//         std::cout << "PositionAndDimensionFilter constructed" << std::endl;
}

PositionAndDimensionFilter::PositionAndDimensionFilter(const PositionAndDimensionFilter& orig) : mhf::IStateEstimator(orig), 
t_last_update_(orig.t_last_update_),
t_last_propagation_(orig.t_last_propagation_),
properties_(0)
// properties_(0)
//kalman_filter_(0),
//fixed_pdf_(0),
//max_acceleration_(orig.max_acceleration_),
//fixed_pdf_cov_(orig.fixed_pdf_cov_ ), 
//kalman_timeout_(orig.kalman_timeout_),
//ProcessModel_(orig.ProcessModel_)
{
/*
    if (orig.fixed_pdf_) {
        fixed_pdf_ = orig.fixed_pdf_->CloneMethod();
    }

    if (orig.kalman_filter_) {
        kalman_filter_ = new KalmanFilter(*orig.kalman_filter_);
    }
    */
// std::cout << "orig.properties_ = " << orig.properties_ << std::endl;
    if (orig.properties_) 
    {
//             std::cout << "printProperties: " << std::endl;
//             orig.properties_->printProperties();
        properties_ = new tracking::FeatureProperties(*orig.properties_);
    }
//     std::cout << "PositionAndDimensionFilter copy constructed" << std::endl;
}

PositionAndDimensionFilter::~PositionAndDimensionFilter() {
//    delete kalman_filter_;
    //delete fixed_pdf_;
        delete properties_;
        
}

PositionAndDimensionFilter* PositionAndDimensionFilter::clone() const {
//         std::cout << "PositionAndDimensionFilter: going to clone" << std::endl;
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

        
//         std::cout << "PositionAndDimensionFilter::update start "  << std::endl;
    if (z->type() == pbl::PDF::HYBRID) {
        std::shared_ptr<const pbl::Hybrid> H = pbl::PDFtoHybrid(z);
     
       
         
        if (!properties_) 
        {
            properties_ = new tracking::FeatureProperties();
            properties_->setMeasuredFeatureProperties(H);
//             std::cout << "PositionAndDimensionFilter::update values initialized. properties = " << std::endl;
//              properties_->printProperties();
        }
       else 
       {     
            tracking::FeatureProperties measuredProperties; 
            measuredProperties.setMeasuredFeatureProperties(H);
            
//             std::cout << "Hybrid: measuredProperties = " << std::endl;
//             measuredProperties.printProperties();
               
            // properties_->updateProbabilities ( measuredProb ); andere filter!!?!
         /*   int rectangle_total_statesize = RECTANGLE_MEASURED_STATE_SIZE + RECTANGLE_MEASURED_DIM_STATE_SIZE;
            int circle_total_statesize = CIRCLE_MEASURED_STATE_SIZE + CIRCLE_DIM_STATE_SIZE;
            pbl::Matrix RmRectangle = G->getCovariance().submat( 0, 0, rectangle_total_statesize - 1, rectangle_total_statesize - 1 );
            pbl::Matrix RmCircle = G->getCovariance().submat( rectangle_total_statesize, rectangle_total_statesize, rectangle_total_statesize + circle_total_statesize - 1, rectangle_total_statesize + circle_total_statesize - 1);
       
            pbl::Vector zmRectangle = G->getMean().subvec(0, rectangle_total_statesize - 1);
            pbl::Vector zmCircle = G->getMean().subvec(rectangle_total_statesize, rectangle_total_statesize + circle_total_statesize - 1);
        */
//             std::cout << "PositionAndDimensionFilter::update rectangle"<< std::endl;

// std::cout << "measuredProperties.rectangle_.getState() = " << measuredProperties.rectangle_.getState() << std::endl;
// std::cout << "measuredProperties.rectangle_.getCovariance() = " << measuredProperties.rectangle_.getCovariance() << std::endl;
pbl::Vector z_kRectangle = measuredProperties.rectangle_.get_H()*measuredProperties.rectangle_.getState();
pbl::Vector z_kCircle = measuredProperties.circle_.get_H()*measuredProperties.circle_.getState();
// std::cout << "PositionAndDimensionFilter, measuredProperties: z_kRectangle.t() = " << z_kRectangle.t() << std::endl;
// std::cout << "PositionAndDimensionFilter, measuredProperties: z_kCircle.t() = " << z_kCircle.t() << std::endl;

            properties_->updateRectangleFeatures( measuredProperties.rectangle_.getCovariance(), z_kRectangle );
//              std::cout << "PositionAndDimensionFilter::update circle" << std::endl;
            properties_->updateCircleFeatures( measuredProperties.circle_.getCovariance(), z_kCircle );
//             std::cout << "PositionAndDimensionFilter::update updateProbabilities." << std::endl;
            properties_->updateProbabilities(measuredProperties.featureProbabilities_);
//             std::cout << "PositionAndDimensionFilter::update finished. Updated properties = " << std::endl;
//             properties_->printProperties();
            
//             std::cout << "Hybrid: updatedProperties = " << std::endl;
//             properties_->printProperties();
        }
        
    } else {
        printf("PositionAndDimensionFilter can only be updated with Hybrids.\n");
    }
    
//     std::cout << "PositionAndDimensionFilter::update end "  << std::endl;
}

void PositionAndDimensionFilter::reset() {
    delete properties_;
    properties_ = 0;
}

std::shared_ptr<const pbl::PDF> PositionAndDimensionFilter::getValue() const {  
   if(properties_)
   {
         /*  observedProperties_->clear();
           std::shared_ptr<pbl::Gaussian> rectPDF = std::make_shared<pbl::Gaussian>( properties_->rectangle_.observedRectangle2PDF() );
           std::shared_ptr<pbl::Gaussian> circPDF = std::make_shared<pbl::Gaussian>( properties_->circle_.observedCircle2PDF() );
           
           observedProperties_->addComponent(rectPDF, properties_->featureProbabilities_.get_pRectangle());
           observedProperties_->addComponent(circPDF, properties_->featureProbabilities_.get_pCircle());
           */
//          std::cout << "PositionAndDimensionFilter::getValue(): ";
//          properties_->printProperties();
         
           return properties_->getPDFSmall();
           
/*           
           pbl::Matrix zeroMatrix(rectPDF.getMean().n_rows,circPDF.getMean().n_cols);
           zeroMatrix.zeros();

           pbl::Matrix fusedMean = arma::join_cols(arma::join_rows(rectPDF.getMean(),zeroMatrix),arma::join_rows(zeroMatrix.t(),circPDF.getMean())); 
           pbl::Matrix fusedCovariance = arma::join_cols(arma::join_rows(rectPDF.getCovariance(),zeroMatrix),arma::join_rows(zeroMatrix.t(),circPDF.getCovariance())); 
   
           observedProperties_->setMean(fusedMean);
           observedProperties_->setCovariance(fusedMean);
           */
          // return observedProperties_;
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

/*bool PositionAndDimensionFilter::setParameter(const std::string& param, bool b) {
    return false;
}
*/

bool PositionAndDimensionFilter::setParameter(const std::string& param, bool b) {
    return false;
}

bool PositionAndDimensionFilter::setParameter(const std::string &param, double v) {
   /* if (param == "max_acceleration") {
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
    }*/
    return true;
}

bool PositionAndDimensionFilter::setParameter(const std::string& param, const std::string& s)
{
      /*  if (param == "process_model") 
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
        }*/
         return false;
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( PositionAndDimensionFilter, mhf::IStateEstimator )