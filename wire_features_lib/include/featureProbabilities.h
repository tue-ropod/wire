#ifndef WIRE_FEATURE_PROBABILITIES_H_
#define WIRE_FEATURE_PROBABILITIES_H_

#include "problib/conversions.h"
#include "problib/datatypes.h"
#include "geolib/datatypes.h"

#define MIN_PROB_OBJECT                  0.05            // [-]

namespace tracking
{

class FeatureProbabilities
{
  public:
    std::shared_ptr<pbl::PMF> pmf_;

    FeatureProbabilities()
    {
        // Initialize with 50/50 probabilities
        pmf_ = std::make_shared<pbl::PMF>();
        pmf_->setDomainSize ( 2 );
        pmf_->setProbability ( "Rectangle", 0.5 );
        pmf_->setProbability ( "Circle", 0.5 );
    };

    void setProbabilities ( float pRectangle_in, float pCircle_in ) {
        pmf_->setProbability ( "Rectangle", pRectangle_in );
        pmf_->setProbability ( "Circle", pCircle_in );
    };

    double get_pRectangle() const 
    {
        double out = pmf_->getProbability ( "Rectangle" );
        return out;
    };

    double get_pCircle() const 
    {
        double out = pmf_->getProbability ( "Circle" );
        return out;
    };

    int getDomainSize (){ return pmf_->getDomainSize(); }
    
    bool setMeasurementProbabilities ( float errorRectangleSquared, float errorCircleSquared, float circleDiameter, float typicalCorridorWidth );

    void update ( float pRectangle_measured, float pCircle_measured );
    
    void update ( FeatureProbabilities& featureProbabilities_in );
    
    std::shared_ptr<pbl::PMF> getValue() { return pmf_; };
    
    void setValue(const pbl::PMF& pmf) { pmf_ = std::make_shared<pbl::PMF>(pmf); };
};

}
#endif