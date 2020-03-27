#ifndef WIRE_FEATURES_CIRCLE_H_
#define WIRE_FEATURES_CIRCLE_H_

#include "problib/conversions.h"
#include "problib/datatypes.h"
#include "geolib/datatypes.h"

#include <visualization_msgs/Marker.h>
#include "tf/transform_datatypes.h"

#include "local_params.h"

#define CIRCLE_STATE_SIZE                 4               // [-]
#define CIRCLE_MEASURED_STATE_SIZE        2               // [-]
#define CIRCLE_DIM_STATE_SIZE             1               // [-]
#define CIRCLE_MEASURED_DIM_STATE_SIZE    1               // [-]

namespace tracking
{
    struct circleMapping {
        unsigned int x_PosVelRef = 0, y_PosVelRef = 1, xVel_PosVelRef = 2, yVel_PosVelRef = 3;//, xAccel_PosVelRef = 4, yAccel_PosVelRef = 5;
        unsigned int r_dimRef = 0;      
        unsigned int x_zRef = 0, y_zRef = 1, radius_zRef = 2;
};

extern struct circleMapping CM;

class Circle
{
  public:
    float x_, y_, z_, roll_, pitch_, yaw_, xVel_, yVel_, radius_; //xAccel_;//, yAccel_, radius_; // x, y, z-positions, roll, pitch, yaw and radius of circle
    pbl::Matrix P_PosVel_, Pdim_; // estimated covariance for state = [x, y, xVel, yVel] ^T and the radius
    
    /* observation model*/
    pbl::Matrix H_PosVel_, H_dim_, H_;
   
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

    bool isValid();
};

}

#endif