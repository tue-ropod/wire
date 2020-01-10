#ifndef WIRE_FEATURE_PROPERTIES_H_
#define WIRE_FEATURE_PROPERTIES_H_

//#include <Eigen/Dense>
//#include <eigen3/Eigen/Dense>

#include "problib/conversions.h"
#include "problib/datatypes.h"
#include "geolib/datatypes.h"

#include <visualization_msgs/Marker.h>
#include "tf/transform_datatypes.h"

//#define ARMA_USE_LAPACK

#define MARKER_TIMEOUT_TIME              0.5             // [s]
#define MIN_PROB_OBJECT                  0.05            // [-]
#define MARGIN_RECTANGLE_INTERCHANGE     30*M_PI/180     // [rad]

#define RECTANGLE_STATE_SIZE              6               // [-]
#define RECTANGLE_MEASURED_STATE_SIZE     3               // [-]
#define RECTANGLE_DIM_STATE_SIZE          2               // [-]
#define RECTANGLE_MEASURED_DIM_STATE_SIZE 2               // [-]
#define CIRCLE_STATE_SIZE                  4               // [-]
#define CIRCLE_MEASURED_STATE_SIZE        2               // [-]
#define CIRCLE_DIM_STATE_SIZE              1               // [-]
#define CIRCLE_MEASURED_DIM_STATE_SIZE     1               // [-]

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

class Circle
{
    float x_, y_, z_, roll_, pitch_, yaw_, xVel_, yVel_, radius_; //xAccel_;//, yAccel_, radius_; // x, y, z-positions, roll, pitch, yaw and radius of circle
    pbl::Matrix P_PosVel_, Pdim_; // estimated covariance for state = [x, y, xVel, yVel] ^T and the radius
    
    /* observation model*/
    pbl::Matrix H_PosVel_, H_dim_, H_;
   
  public:
    Circle();
    
    void setMeasuredCircle(  std::shared_ptr<const pbl::Gaussian> Gmeasured);
    
    void setObservedCircle( std::shared_ptr< const pbl::Gaussian> Gobserved);
    
    void setProperties ( float x, float y, float z, float xVel, float yVel, float roll, float pitch, float yaw, float radius );
    
    float get_x()                                { return x_; } ;
    float get_y()                                { return y_; } ;
    float get_z()                                { return z_; } ;
    float get_roll()                             { return roll_; } ;
    float get_pitch()                            { return pitch_; } ;
    float get_yaw()                              { return yaw_; } ;
    float get_xVel()                             { return xVel_; } ;
    float get_yVel()                             { return yVel_; } ;
    float get_radius()                           { return radius_; } ;
    pbl::Matrix get_P_PosVel()                   { return P_PosVel_; } ;
    pbl::Matrix get_Pdim()                       { return Pdim_; } ;
    pbl::Matrix get_H_PosVel()                   { return H_PosVel_; } ;
    pbl::Matrix get_H_dim()                      { return H_dim_; } ;
    pbl::Matrix get_H()                          { return H_; } ;
    
    void set_x          ( float x )              { x_     = x; } ;
    void set_y          ( float y )              { y_     = y; } ;
    void set_z          ( float z )              { z_     = z; } ;
    void set_roll       ( float roll )           { roll_  = roll; } ;
    void set_pitch      ( float pitch )          { pitch_ = pitch; } ;
    void set_yaw        ( float yaw )            { yaw_   = yaw; } ;
    void set_xVel       ( float xVel )           { xVel_  = xVel; } ;
    void set_yVel       ( float yVel )           { yVel_  = yVel; } ;
    void set_radius     ( float radius )         { radius_ = radius; } ;
    void set_P_PosVel   ( pbl::Matrix P )        { P_PosVel_ = P; } ;
    void set_Pdim       ( pbl::Matrix Pdim )     { Pdim_ = Pdim; } ;

    void setMarker ( visualization_msgs::Marker& marker, unsigned int ID );
    
    void setMarker ( visualization_msgs::Marker& marker, unsigned int ID, std_msgs::ColorRGBA color );
    
    void setTranslationalVelocityMarker( visualization_msgs::Marker& marker, unsigned int ID );
    
    geo::Pose3D getPose() {geo::Pose3D pose(x_, y_, z_, roll_, pitch_,yaw_); return pose; };
    
    std::vector< geo::Vec2f > convexHullPoints(unsigned int nPoints);

    float predictX( float dt );
    
    float predictY( float dt );
    
    void predictPos( float* predictedX, float* predictedY, float dt );
    
    void predictAndUpdatePos( float dt );
    
    pbl::Vector getState();
    
    pbl::Matrix getCovariance();
    
    pbl::Gaussian circle2PDF();
    
    pbl::Gaussian observedCircle2PDF();
    
    void printProperties();
};

class Rectangle
{
    float x_, y_, z_, w_, d_, h_, roll_, pitch_, yaw_, xVel_, yVel_, yawVel_; // x, y of center, width, height and rotation of rectangle
    pbl::Matrix P_PosVel_, Pdim_;
        
    /* observation model*/
    pbl::Matrix H_PosVel_, H_dim_, H_;
        
    
    //pbl::Matrix zeroMatrix(H_PosVel_.n_rows,H_dim_.n_cols);
    //pbl::Matrix H_ = arma::join_cols(arma::join_rows(H_PosVel_,zeroMatrix),arma::join_rows(zeroMatrix.t(), H_dim_));     
    
  public:
    Rectangle();
    
    void setMeasuredRectangle( std::shared_ptr< const pbl::Gaussian> Gmeasured);
    
    void setObservedRectangle( std::shared_ptr< const pbl::Gaussian> Gobserved);
    
    void setValues ( float x, float y, float z, float w, float d, float h, float roll, float pitch, float yaw );
    
    float get_x()                       { return x_; } ;
    float get_y()                       { return y_; } ;
    float get_z()                       { return z_; } ;
    float get_w()                       { return w_; } ;
    float get_d()                       { return d_; } ;
    float get_h()                       { return h_; } ;
    float get_roll()                    { return roll_; } ;
    float get_pitch()                   { return pitch_; } ;
    float get_yaw()                     { return yaw_; } ;
    float get_xVel()                    { return xVel_; } ;
    float get_yVel()                    { return yVel_; } ;
    float get_yawVel()                  { return yawVel_; } ;
    pbl::Matrix get_P_PosVel()          { return P_PosVel_; } ;
    pbl::Matrix get_Pdim()              { return Pdim_; } ;
    pbl::Matrix get_H_PosVel()          { return H_PosVel_; } ;
    pbl::Matrix get_H_dim()             { return H_dim_; } ;
    pbl::Matrix get_H()                 { return H_; } ;
    
    geo::Pose3D getPose() {geo::Pose3D pose(x_, y_, z_, roll_, pitch_,yaw_); return pose; };
    
    void set_x          ( float x )             { x_     = x; } ;
    void set_y          ( float y )             { y_     = y; } ;
    void set_z          ( float z )             { z_     = z; } ;
    void set_w          ( float w )             { w_     = w; } ;
    void set_d          ( float d )             { d_     = d; } ;
    void set_h          ( float h )             { h_     = h; } ;
    void set_roll       ( float roll )          { roll_  = roll; } ;
    void set_pitch      ( float pitch )         { pitch_ = pitch; } ;
    void set_yaw        ( float yaw )           { yaw_   = yaw; } ;
    void set_xVel       ( float xVel )          { xVel_  = xVel; } ;
    void set_yVel       ( float yVel )          { yVel_  = yVel; } ;
    void set_yawVel     ( float yawVel )        { yawVel_  = yawVel; } ;
    void set_P_PosVel   ( pbl::Matrix P )       { P_PosVel_     = P; } ;
    void set_Pdim       ( pbl::Matrix Pdim )    { Pdim_     = Pdim; } ;

    void setMarker ( visualization_msgs::Marker& marker, unsigned int ID );
    
    void setMarker ( visualization_msgs::Marker& marker, unsigned int ID, std_msgs::ColorRGBA color);
    
    void setMarker ( visualization_msgs::Marker& marker, unsigned int ID, std_msgs::ColorRGBA color, std::string ns );
    
    void setTranslationalVelocityMarker( visualization_msgs::Marker& marker, unsigned int ID );
    
    void setRotationalVelocityMarker( visualization_msgs::Marker& marker, unsigned int ID );
    
    std::vector<geo::Vec2f> determineCorners( float associationDistance);
    
    std::vector<geo::Vec2f> determineCenterpointsOfEdges ( );
    
//     std::vector<geo::Vec2f> determinePointsOfSquare ( float associationDistance, float rotation );
    
    float predictX( float dt );
    
    float predictY( float dt );
    
    float predictYaw( float dt );
    
    void predictPos( float* predictedX, float* predictedY, float* predictedYaw, float dt );
    
    void predictAndUpdatePos( float dt );
    
    bool switchDimensions( float measuredYaw);
    
    void interchangeRectangleFeatures();
    
 //   void setState( float posX, float posY, float posYaw, float xVel, float yVel, float yawVel, float width, float depth );
    
    pbl::Gaussian rectangle2PDF();
    
    pbl::Gaussian observedRectangle2PDF(); // TODO proper naming -> rectangle2PDFSmall
    
    pbl::Vector getState();

    pbl::Matrix getCovariance();
    
    void printProperties();
};


// Probabilities
class FeatureProbabilities
{
  public:
    std::shared_ptr<pbl::PMF> pmf_;

    FeatureProbabilities()
    {
        pmf_ = std::make_shared<pbl::PMF>();
//    FeatureProbabilities ( void ) { // Initialize with 50/50 probabilities
        pmf_->setDomainSize ( 2 );
        pmf_->setProbability ( "Rectangle", 0.5 );
        pmf_->setProbability ( "Circle", 0.5 );
    };

    void setProbabilities ( float pRectangle_in, float pCircle_in ) {
        pmf_->setProbability ( "Rectangle", pRectangle_in );
        pmf_->setProbability ( "Circle", pCircle_in );
    };

    double get_pRectangle() const {
            double out = pmf_->getProbability ( "Rectangle" );
        return out;
    } ;

    double get_pCircle() const {
//            std::cout << "Feature prob: ptr = " << this << "\t";
            double out = pmf_->getProbability ( "Circle" );
        return out;
    } ;

    int getDomainSize (){
            return pmf_->getDomainSize();
            }
    bool setMeasurementProbabilities ( float errorRectangleSquared, float errorCircleSquared, float circleDiameter, float typicalCorridorWidth );

    void update ( float pRectangle_measured, float pCircle_measured );
    
    void update ( FeatureProbabilities& featureProbabilities_in );
    
    std::shared_ptr<pbl::PMF> getValue() { return pmf_; };
    
    void setValue(const pbl::PMF& pmf) { pmf_ = std::make_shared<pbl::PMF>(pmf); };
    
    //PMF::PMF(const PMF& pmf) : PDF(1, PDF::DISCRETE), ptr_(pmf.ptr_) {
//}
};

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
   
   ~FeatureProperties ( ) { // Initialize with 50/50 probabilities unless otherwise indicated
    //  delete featureProbabilities_;
   };

    FeatureProperties ( const FeatureProperties* orig ) : 
    featureProbabilities_(orig->featureProbabilities_),
    circle_ (orig->circle_),
    rectangle_(orig->rectangle_),
    nMeasurements_(orig->nMeasurements_)
  //  observedProperties_ (orig->observedProperties_)
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
    
};

}

#endif