#include "circle.h"

namespace tracking{

struct circleMapping CM;

Circle::Circle():   H_PosVel_( arma::eye(CIRCLE_MEASURED_STATE_SIZE, CIRCLE_STATE_SIZE) ), 
                    H_dim_ (arma::eye(CIRCLE_MEASURED_DIM_STATE_SIZE, CIRCLE_DIM_STATE_SIZE) )
{
    float notANumber = 0.0/0.0;
    P_PosVel_ = arma::eye( CIRCLE_STATE_SIZE, CIRCLE_STATE_SIZE );
    Pdim_= arma::eye( CIRCLE_DIM_STATE_SIZE, CIRCLE_DIM_STATE_SIZE ); 
    this->setProperties( notANumber, notANumber, notANumber, 0.0, 0.0, notANumber, notANumber, notANumber, notANumber ); // Produces NaN values, meaning that the properties are not initialized yet

    pbl::Matrix zeroMatrix(H_PosVel_.n_rows,H_dim_.n_cols);
    pbl::Matrix zeroMatrix2(H_dim_.n_rows, H_PosVel_.n_cols);
    zeroMatrix.zeros();
    zeroMatrix2.zeros();

    H_ = arma::join_cols(arma::join_rows(H_PosVel_,zeroMatrix),arma::join_rows(zeroMatrix2, H_dim_));     
}

void Circle::setMeasuredCircle( std::shared_ptr<const pbl::Gaussian> Gmeasured)
{        
        this->setProperties( Gmeasured->getMean().at(CM.x_zRef),  // x
                             Gmeasured->getMean().at(CM.y_zRef), // y
                             0.3, // z TODO TEMP
                             0.0, //G->getMean().at(CM.xVel_PosVelRef), // xvel
                             0.0, //G->getMean().at(CM.yVel_PosVelRef), // yvel
                             0.0, // roll
                             0.0, // pitch
                             0.0, // yaw
                             Gmeasured->getMean().at(CM.radius_zRef)  ); // radius
           
         P_PosVel_ = Gmeasured->getCovariance().submat(0, 0, CIRCLE_MEASURED_STATE_SIZE - 1, CIRCLE_MEASURED_STATE_SIZE - 1);
         Pdim_     = Gmeasured->getCovariance().submat(CIRCLE_MEASURED_STATE_SIZE,
                                                       CIRCLE_MEASURED_STATE_SIZE,
                                                       CIRCLE_MEASURED_STATE_SIZE + CIRCLE_MEASURED_DIM_STATE_SIZE - 1,
                                                       CIRCLE_MEASURED_STATE_SIZE + CIRCLE_MEASURED_DIM_STATE_SIZE - 1);
         
         P_PosVel_ = arma::eye( CIRCLE_STATE_SIZE, CIRCLE_STATE_SIZE ); 
         Pdim_ = arma::eye( CIRCLE_DIM_STATE_SIZE, CIRCLE_DIM_STATE_SIZE ); 
        
         P_PosVel_.submat(0, 0, CIRCLE_MEASURED_STATE_SIZE - 1, CIRCLE_MEASURED_STATE_SIZE - 1) = 
                 Gmeasured->getCovariance().submat(0, 0, CIRCLE_MEASURED_STATE_SIZE - 1, CIRCLE_MEASURED_STATE_SIZE - 1);
        
         Pdim_.submat(0, 0, CIRCLE_MEASURED_DIM_STATE_SIZE - 1, CIRCLE_MEASURED_DIM_STATE_SIZE - 1) =
                 Gmeasured->getCovariance().submat(CIRCLE_MEASURED_STATE_SIZE, 
                                                   CIRCLE_MEASURED_STATE_SIZE,
                                                   CIRCLE_MEASURED_STATE_SIZE + CIRCLE_MEASURED_DIM_STATE_SIZE - 1,
                                                   CIRCLE_MEASURED_STATE_SIZE + CIRCLE_MEASURED_DIM_STATE_SIZE - 1);
}

void Circle::setObservedCircle( std::shared_ptr<const pbl::Gaussian> Gobserved)
{       
        set_x      ( Gobserved->getMean().at(CM.x_PosVelRef) );
        set_y      ( Gobserved->getMean().at(CM.y_PosVelRef) );
        set_z      ( 0.3 );// z TODO TEMP
        
        set_roll   ( 0.0 );
        set_pitch  ( 0.0 );
        set_yaw    ( 0.0 );
        
        set_xVel   ( Gobserved->getMean().at(CM.xVel_PosVelRef) );
        set_yVel   ( Gobserved->getMean().at(CM.yVel_PosVelRef) );

        set_radius ( Gobserved->getMean().at(CIRCLE_STATE_SIZE + CM.r_dimRef) );
        
        P_PosVel_ = Gobserved->getCovariance().submat(0, 0, CIRCLE_STATE_SIZE - 1, CIRCLE_STATE_SIZE - 1);
        Pdim_     = Gobserved->getCovariance().submat(CIRCLE_STATE_SIZE,
                                                      CIRCLE_STATE_SIZE,
                                                      CIRCLE_STATE_SIZE + CIRCLE_DIM_STATE_SIZE - 1,
                                                      CIRCLE_STATE_SIZE + CIRCLE_DIM_STATE_SIZE - 1);
}

void Circle::setProperties ( float x, float y, float z, float xVel, float yVel, float roll, float pitch, float yaw, float radius )
{
    x_ = x;
    y_ = y;
    z_ = z;
    xVel_ = xVel;
    yVel_ = yVel;
    roll_ = roll;
    pitch_ = pitch;
    yaw_ = yaw;
    radius_ = radius;
}

void Circle::setMarker ( visualization_msgs::Marker& marker , unsigned int ID )
{
   std_msgs::ColorRGBA color;
   color.a = 0.5;
   color.r = 0.0;
   color.g = 1.0;
   color.b = 0.0;
   
   this->setMarker ( marker, ID, color ); 
}

void Circle::setMarker ( visualization_msgs::Marker& marker, unsigned int ID, std_msgs::ColorRGBA color )
{
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "Position Marker";
    marker.id = ID;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x_;
    marker.pose.position.y = y_;
    marker.pose.position.z = z_;
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( roll_, pitch_, yaw_ );
    marker.scale.x = 2*radius_;
    marker.scale.y = 2*radius_;
    marker.scale.z = 0.1;
    
    marker.color = color;

    marker.lifetime = ros::Duration ( MARKER_TIMEOUT_TIME );
}

void Circle::setTranslationalVelocityMarker( visualization_msgs::Marker& marker, unsigned int ID )
{
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "Translational Velocity Marker";
    marker.id = ID;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x_;
    marker.pose.position.y = y_;
    marker.pose.position.z = z_;

    // Pivot point is around the tip of its tail. Identity orientation points it along the +X axis. 
    // scale.x is the arrow length, scale.y is the arrow width and scale.z is the arrow height.     
    float rollVel = 0.0;
    float pitchVel = 0.0;
    float yawVel = atan2( yVel_, xVel_ );
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollVel, pitchVel, yawVel );

    marker.scale.x = sqrt( pow(xVel_, 2.0) + pow(yVel_, 2.0) );
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    
    marker.lifetime = ros::Duration ( MARKER_TIMEOUT_TIME );
}

std::vector< geo::Vec2f > Circle::convexHullPoints(unsigned int nPoints)
{
  std::vector< geo::Vec2f > Points(nPoints);
  float deltaAngle = 2*M_PIl / nPoints;
  
  for(unsigned int ii = 0; ii < nPoints; ii++)
  {
    float angle = ii*deltaAngle;
    Points[ii].x = x_ + radius_*cos(angle);
    Points[ii].y = y_ + radius_*sin(angle);
  }
  
  return Points;
}

float Circle::predictX( float dt )
{
        return x_ + dt * xVel_;
}

float Circle::predictY( float dt )
{
        return y_ + dt * yVel_;
}

void Circle::predictPos( float* predictedX, float* predictedY, float dt )
{
        *predictedX = predictX( dt );
        *predictedY = predictY( dt );
}

void Circle::predictAndUpdatePos( float dt )
{
        x_ = predictX( dt );
        y_ = predictY( dt );
}

pbl::Vector Circle::getState()
{
        pbl::Vector state(CIRCLE_STATE_SIZE + CIRCLE_DIM_STATE_SIZE);
        state(0) = x_;
        state(1) = y_;
        state(2) = xVel_;
        state(3) = yVel_;
        state(4) = radius_;
 
        return state;  
}

pbl::Matrix Circle::getCovariance( )
{
        pbl::Matrix zeroMatrix(P_PosVel_.n_rows,Pdim_.n_cols);
        zeroMatrix.zeros();
        pbl::Matrix covariance =arma::join_cols(arma::join_rows(P_PosVel_,zeroMatrix),arma::join_rows(zeroMatrix.t(),Pdim_));    
        return covariance;  
}

pbl::Gaussian Circle::circle2PDF()
{
        pbl::Gaussian G(CIRCLE_STATE_SIZE + CIRCLE_DIM_STATE_SIZE);
        G.setMean( getState() );
        G.setCovariance( getCovariance() );       
        return G;
}

pbl::Gaussian Circle::observedCircle2PDF()
{       
        pbl::Gaussian G_observed(CIRCLE_MEASURED_STATE_SIZE + CIRCLE_MEASURED_DIM_STATE_SIZE);
        G_observed.setMean( H_*getState() );
        G_observed.setCovariance( H_*getCovariance()*H_.t() );
        
        return G_observed;
}

void Circle::printProperties ( )
{
    std::cout << "Circle prop : " ;
    std::cout << "x_ = " << x_;
    std::cout << " y_ = " << y_;
    std::cout << " xVel_ = " << xVel_;
    std::cout << " yVel_ = " << yVel_;
    std::cout << " roll_ = " << roll_;
    std::cout << " pitch_ = " << pitch_;
    std::cout << " yaw_ = " << yaw_;
    std::cout << " radius = " << radius_ ;
    std::cout << " P_ = " << P_PosVel_ << std::endl;
    std::cout << "Pdim_ = " << Pdim_ << std::endl;
}

bool Circle::isValid() 
{
        if(x_ != x_ || y_ != y_ || z_ != z_ || roll_ != roll_ || pitch_ != pitch_ || yaw_ != yaw_ || xVel_ != xVel_ ||
        yVel_ != yVel_ || radius_ != radius_ )
        {
                return false;
        } else {
                return true;
        }
}

}