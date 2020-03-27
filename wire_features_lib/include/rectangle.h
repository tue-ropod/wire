#ifndef WIRE_FEATURES_RECTANGLE_H_
#define WIRE_FEATURES_RECTANGLE_H_

#include "problib/conversions.h"
#include "problib/datatypes.h"
#include "geolib/datatypes.h"

#include <visualization_msgs/Marker.h>
#include "tf/transform_datatypes.h"

#include "local_params.h"

#define RECTANGLE_STATE_SIZE              6               // [-]
#define RECTANGLE_MEASURED_STATE_SIZE     3               // [-]
#define RECTANGLE_DIM_STATE_SIZE          2               // [-]
#define RECTANGLE_MEASURED_DIM_STATE_SIZE 2               // [-]

#define MARGIN_RECTANGLE_INTERCHANGE     30*M_PI/180     // [rad]

namespace tracking
{
    struct rectangleMapping {
    unsigned int x_PosVelRef = 0, y_PosVelRef = 1, yaw_PosVelRef = 2, xVel_PosVelRef = 3, yVel_PosVelRef = 4, yawVel_PosVelRef = 5;//, width_PosVelRef, depth_PosVelRef;
    unsigned int width_dimRef = 0, depth_dimRef = 1;
    unsigned int x_zRef = 0, y_zRef = 1, yaw_zRef = 2, width_zRef = 3, depth_zRef = 4;
};

    extern struct rectangleMapping RM;

class Rectangle
{
    public:
    float x_, y_, z_, w_, d_, h_, roll_, pitch_, yaw_, xVel_, yVel_, yawVel_; // x, y of center, width, height and rotation of rectangle
    pbl::Matrix P_PosVel_, Pdim_;
        
    /* observation model*/
    pbl::Matrix H_PosVel_, H_dim_, H_;   
    
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
    
    float predictX( float dt );
    
    float predictY( float dt );
    
    float predictYaw( float dt );
    
    void predictPos( float* predictedX, float* predictedY, float* predictedYaw, float dt );
    
    void predictAndUpdatePos( float dt );
    
    bool switchDimensions( float measuredYaw);
    
    void interchangeRectangleFeatures();
    
    pbl::Gaussian rectangle2PDF();
    
    pbl::Gaussian observedRectangle2PDF(); // TODO proper naming -> rectangle2PDFSmall
    
    pbl::Vector getState();

    pbl::Matrix getCovariance();
    
    void printProperties();

    bool isValid();
};

}
#endif