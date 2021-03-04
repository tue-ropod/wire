#ifndef WIRE_FEATURE_PROPERTIES_H_
#define WIRE_FEATURE_PROPERTIES_H_

#include "problib/conversions.h"
#include "problib/datatypes.h"
#include "geolib/datatypes.h"

#include "rectangle.h"
#include "circle.h"
#include "featureProbabilities.h"

#define MAX_DIMENSION_UNCERTAINTY         2               // [-]

namespace tracking
{

template <typename T> 
int sgn(T val) 
{
    return (T(0) < val) - (val < T(0));
}

template <typename T> 
void unwrap (T *angleMeasured, T angleReference, T increment)
{
        // Rectangle is symmetric over pi-radians, so unwrap to pi
        T diff = angleReference - *angleMeasured;
        
        int d = diff / (increment);
        *angleMeasured += d*increment;
        
        T r = angleReference - *angleMeasured;
        
        if( fabs(r) > (0.5*increment) )
        {
                *angleMeasured += sgn(r)*increment;
        }
}

class FeatureProperties
{
  public:
    FeatureProbabilities featureProbabilities_; // Probabilities of the features. Is there a need to reset these when there is a switch? Or only when the probability of a feature was low?

    Circle circle_;

    Rectangle rectangle_;
    
    std::shared_ptr<pbl::Hybrid> observedProperties_;
    
    int nMeasurements_;
    
    FeatureProperties ( ) { // Initialize with 50/50 probabilities unless otherwise indicated
      featureProbabilities_.setProbabilities ( 0.5, 0.5 );
      nMeasurements_ = 0;
   };
   
   ~FeatureProperties ( ) {   };

    FeatureProperties ( const FeatureProperties* orig ) : 
    featureProbabilities_(orig->featureProbabilities_),
    circle_ (orig->circle_),
    rectangle_(orig->rectangle_),
    nMeasurements_(orig->nMeasurements_)
    {  
            if (orig->observedProperties_) 
            {
                    observedProperties_ = std::make_shared<pbl::Hybrid>(*orig->observedProperties_);
            }
    };
    
    
    void setObservedFeatureProperties ( std::shared_ptr<const pbl::PDF> observedProperties ); 
     
    void setMeasuredFeatureProperties ( std::shared_ptr<const pbl::PDF>measuredProperties );

    FeatureProbabilities getFeatureProbabilities() const {
        return featureProbabilities_;
    };

    void setFeatureProbabilities ( float pRectangle_in, float pCircle_in ) {
        featureProbabilities_.setProbabilities ( pRectangle_in, pCircle_in );
    };

    void setFeatureProbabilities ( FeatureProbabilities featureProbabilities_in ) {
        featureProbabilities_ = featureProbabilities_in;
    };

    void updateProbabilities ( FeatureProbabilities featureProbabilities_in ) {
        featureProbabilities_.update ( featureProbabilities_in );
    };

    void setCircle ( Circle circle_in ) {
        circle_ = circle_in;
    };

    Circle getCircle() const {
        return circle_;
    };

    Rectangle getRectangle() const {
        return rectangle_;
    };

    void setRectangle ( Rectangle rectangle_in ) {
        rectangle_ = rectangle_in;
    };
    
    int getNMeasurements() const{
            return nMeasurements_;
    };
    
    void setNMeasurements( int nMeasurements ){
            nMeasurements_ = nMeasurements;
    };
    
    void updatePosition();
    
    void propagateCircleFeatures (pbl::Matrix Q_k, float dt);
    
    void updateCircleFeatures( pbl::Matrix R_k, pbl::Vector z_k);
    
    void posVelState2Circle( pbl::Vector x_k_k_PosVel );
    
    void propagateRectangleFeatures (pbl::Matrix Q_k, float dt);
    
    void updateRectangleFeatures( pbl::Matrix R_k, pbl::Vector z_k);
    
    void posVelState2Rectangle( pbl::Vector x_k_k_PosVel );
    
    void correctForDimensions( float deltaWidth, float deltaDepth, float* xMeasured, float* yMeasured, float measuredPosX, float measuredPosY, float modelledPosX, float modelledPosY,  float thetaPred );
    
    void correctPosForDimDiff(float deltaWidth, float deltaDepth, float *deltaX, float *deltaY, float thetaPred);
    
    void printProperties();
    
    std::shared_ptr<pbl::Hybrid> getPDF();
    
    std::shared_ptr<pbl::Hybrid> getPDFSmall();

    bool isValid();
};

}

#endif