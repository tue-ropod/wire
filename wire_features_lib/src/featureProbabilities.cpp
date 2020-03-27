#include "featureProbabilities.h"

namespace tracking{

bool FeatureProbabilities::setMeasurementProbabilities ( float errorRectangleSquared, float errorCircleSquared, float circleDiameter, float typicalCorridorWidth )
{       
    if ( !std::isinf ( errorRectangleSquared ) || !std::isinf ( errorCircleSquared ) )
    {
        float probabilityScaling = 1.0;
        if ( circleDiameter > 0.5*typicalCorridorWidth )
        {
            // Circles with a very large radius could have a smaller error compared to fitting a line. For the type of environment, this is very unlikely.
            // Therefore, the probability of large circles (diameter > typicalCorridorWidth) is reduced in an exponential fashion
            probabilityScaling = std::exp ( -1 / ( 0.5*typicalCorridorWidth ) * ( circleDiameter -0.5*typicalCorridorWidth ) );
        }

        float sum = errorRectangleSquared + errorCircleSquared;
        float pCircle = probabilityScaling * errorRectangleSquared/sum;
        if(pCircle < MIN_PROB_OBJECT || pCircle != pCircle) // smooth out prob such that recovery is easier
        {
                pCircle = MIN_PROB_OBJECT;
        }
        else if(pCircle > 1.0 - MIN_PROB_OBJECT)
        {
                pCircle = 1.0 - MIN_PROB_OBJECT;
        }

        float pRectangle =  1.0 - pCircle;  // Only 2 objects now, so the sum of it equals 1

        pmf_->setProbability ( "Rectangle", pRectangle );
        pmf_->setProbability ( "Circle", pCircle );
        
        return true;
    }
    else
    {
        // Infinity detected on both of the elements: this is the case when the number of points is too small to do a fit. Equal probability set.
        pmf_->setProbability ( "Rectangle", 0.5 );
        pmf_->setProbability ( "Circle", 0.5 );
    
            // TODO if there are enough points for a single fit (probably circle only), is this fit realistic?
            // Acatually, it should be measured if the object which is modelled is realistic by comparing the laser scan with the expected scan based on that object
            
            return false;
    }
}

void FeatureProbabilities::update ( float pRectangle_measured, float pCircle_measured )
{
    std::shared_ptr<pbl::PMF> pmf_measured = pmf_;

    float pCircle = pmf_->getProbability ( "Circle" );            
    
    if(pCircle < MIN_PROB_OBJECT || pCircle != pCircle ) // smooth out prob such that recovery is easier
    {
            pCircle = MIN_PROB_OBJECT;
    }
    else if(pCircle > 1.0 - MIN_PROB_OBJECT)
    {
            pCircle = 1.0 - MIN_PROB_OBJECT;
    }

    float pRectangle =  1.0 - pCircle;  // Only 2 objects now, so the sum of it equals 1

    pmf_measured->setProbability ( "Rectangle", pRectangle_measured );
    pmf_measured->setProbability ( "Circle", pCircle_measured );

    pmf_->update ( pmf_measured );
}

void FeatureProbabilities::update ( FeatureProbabilities& featureProbabilities_in )
{
    pmf_->update ( featureProbabilities_in.pmf_ );
    
    
    float pCircle = pmf_->getProbability ( "Circle" );            
    
    bool updated = false;
    if(pCircle < MIN_PROB_OBJECT ) // smooth out prob such that recovery is easier
    {
            pCircle = MIN_PROB_OBJECT;
            updated = true;
    }
    else if(pCircle > 1.0 - MIN_PROB_OBJECT)
    {
            pCircle = 1.0 - MIN_PROB_OBJECT;
            updated = true;
    }
    
    if(updated)
    {
            float pRectangle =  1.0 - pCircle;  // Only 2 objects now, so the sum of it equals 1
            pmf_->setProbability ( "Rectangle", pRectangle );
            pmf_->setProbability ( "Circle", pCircle );
    }
}

}