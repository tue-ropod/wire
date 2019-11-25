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

#include "PositionAndDimensionFilter.h"
#include "KalmanFilter.h"

Position2Filter::Position2Filter() : 

t_last_update_(0),
t_last_propagation_(0)
//kalman_filter_(0),
//fixed_pdf_(0),
//max_acceleration_(0),
//fixed_pdf_cov_(0),
//kalman_timeout_(0),
//ProcessModel_(ProcessModel::NONE)
{
        std::cout << "PositionAndDimensionFilter constructed" << std::endl;
}

Position2Filter::Position2Filter(const Position2Filter& orig) : mhf::IStateEstimator(orig), 
t_last_update_(orig.t_last_update_),
t_last_propagation_(orig.t_last_propagation_)
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
        if (orig.properties_) {
        properties_ = new tracking::FeatureProperties(*orig.properties_);
    }
    std::cout << "Position2Filter copy constructed" << std::endl;
}

Position2Filter::~Position2Filter() {
//    delete kalman_filter_;
    //delete fixed_pdf_;
        delete properties_;
}

Position2Filter* Position2Filter::clone() const {
        std::cout << "Position2Filter: going to clone" << std::endl;
    return new Position2Filter(*this);
}

void Position2Filter::propagate(const mhf::Time& time) {

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
    diagQRect << Q << Q << Q << 20*Q << 20*Q << 20*Q << Q << Q << arma::endr; // TODO why this difference?
    diagQCircle << Q << Q << Q << Q << Q << arma::endr;
    pbl::Matrix QmRectangle = arma::diagmat( diagQRect );
    pbl::Matrix QmCircle = arma::diagmat( diagQCircle );
   
    properties_->propagateRectangleFeatures( QmRectangle, dt );
    properties_->propagateCircleFeatures( QmCircle, dt);
}

void Position2Filter::update(std::shared_ptr<const pbl::PDF> z, const mhf::Time& time) {

    if (z->type() == pbl::PDF::GAUSSIAN) {
        std::shared_ptr<const pbl::Gaussian> G = pbl::PDFtoGaussian(z);
     
         // properties_->updateProbabilities ( measuredProb ); andere filter!!?!

        if (!properties_) 
        {
            properties_ = new tracking::FeatureProperties( );
            //kalman_filter_->setMaxAcceleration(max_acceleration_);
            //kalman_filter_->init(G);
        }
        //else {
            int rectangle_total_statesize = RECTANGLE_STATESIZE + RECTANGLE_DIM_STATESIZE;
            int circle_total_statesize = CIRCLE_STATESIZE + CIRCLE_DIM_STATESIZE;
            pbl::Matrix RmRectangle = G->getCovariance().submat( 0, 0, rectangle_total_statesize - 1, rectangle_total_statesize - 1 );
            pbl::Matrix RmCircle = G->getCovariance().submat( rectangle_total_statesize, rectangle_total_statesize, rectangle_total_statesize + circle_total_statesize - 1, rectangle_total_statesize + circle_total_statesize - 1);
        
            pbl::Vector zmRectangle = G->getMean().subvec(0, rectangle_total_statesize - 1);
            pbl::Vector zmCircle = G->getMean().subvec(rectangle_total_statesize, rectangle_total_statesize + circle_total_statesize - 1);
            properties_->updateRectangleFeatures( RmRectangle, zmRectangle );
            properties_->updateCircleFeatures( RmCircle, zmCircle);
        //}
        
    } else {
        printf("Position2Filter can only be updated with Gaussians.\n");
    }
    
}

void Position2Filter::reset() {
    delete properties_;
    properties_ = 0;
}

std::shared_ptr<const pbl::PDF> Position2Filter::getValue() const {  
   if(observedProperties_)
   {
           observedProperties_->clear();
           std::shared_ptr<pbl::Gaussian> rectPDF = std::make_shared<pbl::Gaussian>( properties_->rectangle_.observedRectangle2PDF() );
           std::shared_ptr<pbl::Gaussian> circPDF = std::make_shared<pbl::Gaussian>( properties_->circle_.observedCircle2PDF() );
           
           observedProperties_->addComponent(rectPDF, properties_->featureProbabilities_.get_pRectangle());
           observedProperties_->addComponent(circPDF, properties_->featureProbabilities_.get_pCircle());
           
/*           
           pbl::Matrix zeroMatrix(rectPDF.getMean().n_rows,circPDF.getMean().n_cols);
           zeroMatrix.zeros();

           pbl::Matrix fusedMean = arma::join_cols(arma::join_rows(rectPDF.getMean(),zeroMatrix),arma::join_rows(zeroMatrix.t(),circPDF.getMean())); 
           pbl::Matrix fusedCovariance = arma::join_cols(arma::join_rows(rectPDF.getCovariance(),zeroMatrix),arma::join_rows(zeroMatrix.t(),circPDF.getCovariance())); 
   
           observedProperties_->setMean(fusedMean);
           observedProperties_->setCovariance(fusedMean);
           */
           return observedProperties_;
   }
   
    std::cout << "Position2Filter::getValue(): SOMETHINGS WRONG" << std::endl;
}

/*bool Position2Filter::setParameter(const std::string& param, bool b) {
    return false;
}
*/

bool Position2Filter::setParameter(const std::string& param, bool b) {
    return false;
}

bool Position2Filter::setParameter(const std::string &param, double v) {
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

bool Position2Filter::setParameter(const std::string& param, const std::string& s)
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
PLUGINLIB_EXPORT_CLASS( Position2Filter, mhf::IStateEstimator )