#include "wire_state_estimators/featureProperties.h"
#include <boost/iterator/iterator_concepts.hpp>

namespace tracking{
struct rectangleMapping RM;     
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

Rectangle::Rectangle(): H_dim_ ( arma::eye(RECTANGLE_MEASURED_DIM_STATE_SIZE, RECTANGLE_DIM_STATE_SIZE)),
                        H_PosVel_( arma::eye(RECTANGLE_MEASURED_STATE_SIZE, RECTANGLE_STATE_SIZE) )
{
    float notANumber = 0.0/0.0;
    P_PosVel_ = arma::eye( RECTANGLE_STATE_SIZE, RECTANGLE_STATE_SIZE ); 
    Pdim_ = arma::eye( RECTANGLE_DIM_STATE_SIZE, RECTANGLE_DIM_STATE_SIZE ); 
    this->setValues( notANumber, notANumber, notANumber, notANumber, notANumber, notANumber, notANumber, notANumber, notANumber ); // Produces NaN values, meaning that the properties are not initialized yet
    xVel_   = 0.0;
    yVel_   = 0.0;
    yawVel_ = 0.0;
    
    pbl::Matrix zeroMatrix(H_PosVel_.n_rows,H_dim_.n_cols);
    pbl::Matrix zeroMatrix2(H_dim_.n_rows, H_PosVel_.n_cols);
    
    zeroMatrix.zeros();
    zeroMatrix2.zeros();

    H_ = arma::join_cols(arma::join_rows(H_PosVel_,zeroMatrix),arma::join_rows(zeroMatrix2, H_dim_));
}

void Rectangle::setObservedRectangle( std::shared_ptr<const pbl::Gaussian> Gobserved)
{
        set_x      ( Gobserved->getMean().at(RM.x_PosVelRef) );
        set_y      ( Gobserved->getMean().at(RM.y_PosVelRef) );
        set_z      ( 0.3 );// z TODO TEMP
        set_h      ( 0.1 );// h TODO TEMP

        set_roll   ( 0.0 );
        set_pitch  ( 0.0);
        set_yaw    ( Gobserved->getMean().at(RM.yaw_PosVelRef) );

        set_xVel   ( Gobserved->getMean().at(RM.xVel_PosVelRef) );
        set_yVel   ( Gobserved->getMean().at(RM.yVel_PosVelRef) );
        set_yawVel ( Gobserved->getMean().at(RM.yawVel_PosVelRef) );

        set_w      ( Gobserved->getMean().at(RECTANGLE_STATE_SIZE + RM.width_dimRef));
        set_d      ( Gobserved->getMean().at(RECTANGLE_STATE_SIZE + RM.depth_dimRef));
        
        P_PosVel_ = Gobserved->getCovariance().submat(0, 0, RECTANGLE_STATE_SIZE - 1, RECTANGLE_STATE_SIZE - 1);
        Pdim_     = Gobserved->getCovariance().submat(RECTANGLE_STATE_SIZE,
                                                      RECTANGLE_STATE_SIZE,
                                                      RECTANGLE_STATE_SIZE + RECTANGLE_DIM_STATE_SIZE - 1,
                                                      RECTANGLE_STATE_SIZE + RECTANGLE_DIM_STATE_SIZE - 1);
}

void Rectangle::setMeasuredRectangle( std::shared_ptr<const pbl::Gaussian> Gmeasured)
{
        this->setValues( Gmeasured->getMean().at(RM.x_zRef),  // x
                         Gmeasured->getMean().at(RM.y_zRef), // y
                         0.3, // z TODO TEMP
                         Gmeasured->getMean().at(RM.width_zRef), // w
                         Gmeasured->getMean().at(RM.depth_zRef), // d
                         0.1, // h TODO TEMP
                         0.0, // roll
                         0.0, // pitch
                         Gmeasured->getMean().at(RM.yaw_zRef) ); // yaw
  
        
        P_PosVel_ = arma::eye( RECTANGLE_STATE_SIZE, RECTANGLE_STATE_SIZE ); 
        Pdim_ = arma::eye( RECTANGLE_DIM_STATE_SIZE, RECTANGLE_DIM_STATE_SIZE ); 
        
        P_PosVel_.submat(0, 0, RECTANGLE_MEASURED_STATE_SIZE - 1, RECTANGLE_MEASURED_STATE_SIZE - 1) = 
                Gmeasured->getCovariance().submat(0, 0, RECTANGLE_MEASURED_STATE_SIZE - 1, RECTANGLE_MEASURED_STATE_SIZE - 1);
        
        Pdim_.submat(0, 0, RECTANGLE_MEASURED_DIM_STATE_SIZE - 1, RECTANGLE_MEASURED_DIM_STATE_SIZE - 1) =
                 Gmeasured->getCovariance().submat(RECTANGLE_MEASURED_STATE_SIZE, 
                                                   RECTANGLE_MEASURED_STATE_SIZE,
                                                   RECTANGLE_MEASURED_STATE_SIZE + RECTANGLE_MEASURED_DIM_STATE_SIZE - 1,
                                                   RECTANGLE_MEASURED_STATE_SIZE + RECTANGLE_MEASURED_DIM_STATE_SIZE - 1);
}

void Rectangle::setValues ( float x, float y, float z, float w, float d, float h, float roll, float pitch, float yaw )
{
    x_ = x;
    y_ = y;
    z_ = z;
    w_ = w;
    d_ = d;
    h_ = h;
    roll_ = roll;
    pitch_ = pitch;
    yaw_ = yaw;
}

void Rectangle::printProperties ( )
{
    std::cout.precision(4);
    std::cout << "Rect prop = " ;
    std::cout << "x_ = "      << x_;
    std::cout << " y_ = "     << y_;
    std::cout << " z_ = "     << z_;
    std::cout << " w_ = "     << w_;
    std::cout << " d_ = "     << d_;
    std::cout << " h_ = "     << h_;
    std::cout << " xVel_ = "  << xVel_;
    std::cout << " yVel_ = "  << yVel_;
    std::cout << " yawVel_ = "<< yawVel_;
    std::cout << " roll_ = "  << roll_;
    std::cout << " pitch_ = " << pitch_;
    std::cout << " yaw_ = "   << yaw_ << std::endl;
    std::cout << " P_ = " << P_PosVel_ << std::endl;
    std::cout << "Pdim_ = " << Pdim_ << std::endl;
}

bool Rectangle::isValid()
{
        if(x_ != x_ || y_ != y_ || z_ != z_ || w_ != w_ || d_ != d_ || h_ != h_ || xVel_ != xVel_ || yVel_ != yVel_ || 
        yawVel_ != yawVel_ || roll_ != roll_ || pitch_ != pitch_ || yaw_ != yaw_)
        {
                return false;
        } else {
                return true;
        }
}

float Rectangle::predictX( float dt )
{
        return x_ + dt * xVel_;
}

float Rectangle::predictY( float dt )
{
        return y_ + dt * yVel_;
}

float Rectangle::predictYaw( float dt )
{
        return yaw_ + dt * yawVel_;
}

void Rectangle::predictPos( float* predictedX, float* predictedY, float* predictedYaw, float dt )
{
        *predictedX = predictX( dt );
        *predictedY = predictY( dt );
        *predictedYaw = predictYaw( dt );
}

void Rectangle::predictAndUpdatePos( float dt )
{
        x_ = predictX( dt );
        y_ = predictY( dt );
        yaw_ = predictYaw( dt );
}

void Rectangle::setMarker ( visualization_msgs::Marker& marker, unsigned int ID, std_msgs::ColorRGBA color, std::string ns )
{
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = ID;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x_;
    marker.pose.position.y = y_;
    marker.pose.position.z = z_;
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( roll_, pitch_, yaw_ );
    marker.scale.x = w_;
    marker.scale.y = d_;
    marker.scale.z = 0.1;
    marker.color = color;
    marker.lifetime = ros::Duration ( MARKER_TIMEOUT_TIME );
}

void Rectangle::setMarker ( visualization_msgs::Marker& marker , unsigned int ID )
{
   std_msgs::ColorRGBA color;
   color.a = 0.5;
   color.r = 0.0;
   color.g = 1.0;
   color.b = 0.0;
   
   this->setMarker ( marker, ID, color ); 
}

void Rectangle::setMarker ( visualization_msgs::Marker& marker, unsigned int ID, std_msgs::ColorRGBA color)
{
        this->setMarker ( marker, ID, color, "Position Marker" ); 
}

void Rectangle::setTranslationalVelocityMarker( visualization_msgs::Marker& marker, unsigned int ID )
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
    
    float rollTranslationalVel = 0.0;
    float pitchTranslationalVel = 0.0;
    float yawTranslationalVel = std::atan2( yVel_, xVel_ );
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollTranslationalVel, pitchTranslationalVel, yawTranslationalVel );

   // Pivot point is around the tip of its tail. Identity orientation points it along the +X axis. 
   // scale.x is the arrow length, scale.y is the arrow width and scale.z is the arrow height.     
    marker.scale.x = std::sqrt( std::pow(xVel_, 2.0) + std::pow(yVel_, 2.0) );
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    
    marker.lifetime = ros::Duration ( MARKER_TIMEOUT_TIME );
}

void Rectangle::setRotationalVelocityMarker( visualization_msgs::Marker& marker, unsigned int ID )
{
    // At the first corner, place an indicator about the rotational vel
    std::vector<geo::Vec2f> corners = determineCorners( 0.0 );
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "Rotational Velocity Marker";
    marker.id = ID;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x = corners[0].x;
    marker.pose.position.y = corners[0].y;
    marker.pose.position.z = z_;

    // Pivot point is around the tip of its tail. Identity orientation points it along the +X axis. 
    // scale.x is the arrow length, scale.y is the arrow width and scale.z is the arrow height.     
    float rollRotVel = 0.0;
    float pitchRotVel = 0.0;
    float yawRotVel = atan2(d_, w_) - M_PI_2;
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollRotVel, pitchRotVel, yaw_ + yawRotVel );
    
    marker.scale.x = ( pow(w_, 2.0) + pow(d_, 2.0) )*yawVel_; // Velocity of the cornerpoint, so scaled with the distance from the center.
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;  
    
    marker.lifetime = ros::Duration ( MARKER_TIMEOUT_TIME );
}

std::vector<geo::Vec2f> Rectangle::determineCorners ( float associationDistance )
{
    float dx = 0.5* ( w_ + associationDistance ); // blow up for associations
    float dy = 0.5* ( d_ + associationDistance );

    float ct = cos ( yaw_ );
    float st = sin ( yaw_ );

    geo::Vec2f originCorner ( x_ + ct*-dx + st* dy, y_ + st*-dx + ct*-dy ); // Rotation matrix @ -x, -y
    geo::Vec2f corner1 (      x_ + ct* dx + st* dy, y_ + st* dx + ct*-dy ); // @ +x, -y
    geo::Vec2f corner2 (      x_ + ct* dx - st* dy, y_ + st* dx + ct* dy ); // @ +x, +y
    geo::Vec2f corner3 (      x_ + ct*-dx - st* dy, y_ + st*-dx + ct* dy ); // @ -x, +y

    std::vector< geo::Vec2f > corners;
    corners.push_back ( originCorner );
    corners.push_back ( corner1 );
    corners.push_back ( corner2 );
    corners.push_back ( corner3 );

    return corners;
}

std::vector<geo::Vec2f> Rectangle::determineCenterpointsOfEdges ( )
{
        float rotWidth = yaw_ + M_PI_4;
        float ct = cos ( rotWidth );
        float st = sin ( rotWidth );
        
        float length = 0.5*d_;
        
        geo::Vec2f originCorner ( x_ + ct* length + st* 0, y_ + st* length + ct*0 );
        geo::Vec2f corner1 (      x_ + ct*-length + st* 0, y_ + st*-length + ct*0 );
        
        length = 0.5*w_;
        geo::Vec2f corner2 (      x_ + ct* 0 - st* length, y_ + st* 0 + ct* length );
        geo::Vec2f corner3 (      x_ + ct* 0 - st*-length, y_ + st* 0 + ct* -length );

        std::vector< geo::Vec2f > corners;
        corners.push_back ( originCorner );
        corners.push_back ( corner1 );
        corners.push_back ( corner2 );
        corners.push_back ( corner3 );

        return corners;  
}

bool Rectangle::switchDimensions( float measuredYaw)
{
        return std::fabs( std::fabs( yaw_ - measuredYaw )- M_PI_2 ) < MARGIN_RECTANGLE_INTERCHANGE;
}
void Rectangle::interchangeRectangleFeatures()
{
        float widthOld = w_;
        w_ = d_;
        d_ = widthOld;
        
       yaw_ += M_PI_2;
       
       float P_depthOld = Pdim_ ( 1, 1 );
       Pdim_( 1, 1) = Pdim_( 0, 0);
       Pdim_( 0, 0) = P_depthOld;
}

pbl::Vector Rectangle::getState()
{
        pbl::Vector state(RECTANGLE_STATE_SIZE + RECTANGLE_DIM_STATE_SIZE);
        state(0) = x_;
        state(1) = y_;
        state(2) = yaw_;
        state(3) = xVel_;
        state(4) = yVel_;
        state(5) = yawVel_;
        state(6) = w_;
        state(7) = d_;
       
        return state;    
}

pbl::Matrix Rectangle::getCovariance( )
{
        pbl::Matrix zeroMatrix(P_PosVel_.n_rows,Pdim_.n_cols);
        zeroMatrix.zeros();       
        pbl::Matrix covariance =arma::join_cols(arma::join_rows(P_PosVel_,zeroMatrix),arma::join_rows(zeroMatrix.t(),Pdim_));    
        return covariance;  
}       

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

void FeatureProperties::correctForDimensions( float deltaWidth, float deltaDepth, float* xMeasured, float* yMeasured, float measuredPosX, float measuredPosY, float modelledPosX, float modelledPosY,  float thetaPred)
{
        float deltaX_Width, deltaY_Width, deltaX_Depth, deltaY_Depth;
        correctPosForDimDiff(deltaWidth, 0, &deltaX_Width, &deltaY_Width, thetaPred );
        correctPosForDimDiff(0, deltaDepth, &deltaX_Depth, &deltaY_Depth, thetaPred );
        
        float distPosWidth2 = pow( modelledPosX + deltaX_Width - measuredPosX, 2.0 ) + pow( modelledPosY + deltaY_Width - measuredPosY, 2.0 );
        float distNegWidth2 = pow( modelledPosX - deltaX_Width - measuredPosX, 2.0 ) + pow( modelledPosY - deltaY_Width - measuredPosY, 2.0 );
        
        float distPosDepth2 = pow( modelledPosX + deltaX_Depth - measuredPosX, 2.0 ) + pow( modelledPosY + deltaY_Depth - measuredPosY, 2.0 );
        float distNegDepth2 = pow( modelledPosX - deltaX_Depth - measuredPosX, 2.0 ) + pow( modelledPosY - deltaY_Depth - measuredPosY, 2.0 );
        
        
        int signWidth =  distPosWidth2 < distNegWidth2 ? 1 : -1;
        int signDepth =  distPosDepth2 < distNegDepth2 ? 1 : -1;
        
        *xMeasured += (signWidth*deltaX_Width + signDepth*deltaX_Depth);
        *yMeasured += (signWidth*deltaY_Width + signDepth*deltaY_Depth);       
}

void FeatureProperties::correctPosForDimDiff(float deltaWidth, float deltaDepth, float *deltaX, float *deltaY, float thetaPred)
{
        //float thetaPred = rectangle_.get_yaw() + dt*rectangle_.get_yawVel();
        
        float pred_st = std::sin( thetaPred );
        float pred_ct = std::cos( thetaPred );
                
        // check in which direction the measured center-point falls in order to determine to right direction of correction in both x and y
        float rotatedX = deltaWidth * pred_ct - deltaDepth * pred_st;
        float rotatedY = deltaWidth * pred_st + deltaDepth * pred_ct;

        *deltaX = 0.5*rotatedX;
        *deltaY =  0.5*rotatedY;
}
void FeatureProperties::setObservedFeatureProperties ( std::shared_ptr<const pbl::PDF> observedProperties )      
{  
        if (observedProperties->type() == pbl::PDF::HYBRID ) 
        {
            std::shared_ptr<const pbl::Hybrid> observedPropertiesHyb = pbl::PDFtoHybrid(observedProperties);

            const std::vector<pbl::Hybrid::distributionStruct> PDFs = observedPropertiesHyb->getPDFS();
            
            std::shared_ptr< const pbl::PDF> rectPDF = PDFs[0].pdf; // TODO proper numbering for conversion
            std::shared_ptr< const pbl::PDF> circPDF = PDFs[1].pdf;
            
             if (rectPDF->type() == pbl::PDF::GAUSSIAN && circPDF->type() == pbl::PDF::GAUSSIAN)// && probabilityPDF->type() == pbl::PDF::DISCRETE ) 
                {
                        std::shared_ptr<const pbl::Gaussian> rectGauss = pbl::PDFtoGaussian(rectPDF);
                        std::shared_ptr<const pbl::Gaussian> circGauss = pbl::PDFtoGaussian(circPDF);

                        rectangle_.setObservedRectangle(rectGauss);
                        circle_.setObservedCircle(circGauss);
                        featureProbabilities_.setProbabilities ( PDFs[0].weight, PDFs[1].weight );
                } else {
                        std::printf("Circle and Rectangle can only be set with Gaussians. \n"); 
                }
        } else {
                
               std::printf("ObservedProperties can only be set with Hybrids.\n"); 
        }
};
     
void FeatureProperties::setMeasuredFeatureProperties ( std::shared_ptr<const pbl::PDF>measuredProperties )
{
        if (measuredProperties->type() == pbl::PDF::HYBRID ) 
        {
            std::shared_ptr<const pbl::Hybrid> measuredPropertiesHyb = pbl::PDFtoHybrid(measuredProperties);

            const std::vector<pbl::Hybrid::distributionStruct> PDFs = measuredPropertiesHyb->getPDFS();
            
            std::shared_ptr< const pbl::PDF> rectPDF = PDFs[0].pdf; // TODO proper numbering for conversion
            std::shared_ptr< const pbl::PDF> circPDF = PDFs[1].pdf;
            
             if (rectPDF->type() == pbl::PDF::GAUSSIAN && circPDF->type() == pbl::PDF::GAUSSIAN)// && probabilityPDF->type() == pbl::PDF::DISCRETE ) 
                {
                        std::shared_ptr<const pbl::Gaussian> rectGauss = pbl::PDFtoGaussian(rectPDF);
                        std::shared_ptr<const pbl::Gaussian> circGauss = pbl::PDFtoGaussian(circPDF);

                        rectangle_.setMeasuredRectangle(rectGauss);
                        circle_.setMeasuredCircle(circGauss);
                        featureProbabilities_.setProbabilities ( PDFs[0].weight, PDFs[1].weight );
                } else {
                        std::printf("Circle and Rectangle can only be set with Gaussians. \n"); 
                }
        } else {
                
               std::printf("ObservedProperties can only be set with Hybrids.\n"); 
        }
};

pbl::Vector kalmanPropagate(pbl::Matrix F, pbl::Matrix *P, pbl::Vector x_k_1_k_1, pbl::Matrix Q)
{   
    pbl::Vector x_k_k_1 = F*x_k_1_k_1;
    *P = F* (*P) * F.t() + Q;
    return x_k_k_1;
}
    
pbl::Vector kalmanUpdate(pbl::Matrix H, pbl::Matrix *P, pbl::Vector x_k_k_1, pbl::Vector z_k, pbl::Matrix R)
{           
    pbl::Matrix I = arma::eye(P->n_rows, P->n_cols);
    pbl::Vector y_k = z_k - H*x_k_k_1;
    pbl::Matrix S_k = H* *P*H.t() + R;
    pbl::Vector K_k = *P*H.t() * inv(S_k);
    pbl::Vector x_k_k = x_k_k_1 + K_k*y_k;
    *P = ( I - K_k*H )* *P;
    
    return x_k_k;
}

void FeatureProperties::propagateRectangleFeatures (pbl::Matrix Q_k, float dt)
{ 
         pbl::Matrix F_PosVel;
         F_PosVel << 1.0 << 0.0 << 0.0 << dt  << 0.0 << 0.0 << arma::endr // x 
                  << 0.0 << 1.0 << 0.0 << 0.0 << dt  << 0.0 << arma::endr // y 
                  << 0.0 << 0.0 << 1.0 << 0.0 << 0.0 << dt  << arma::endr // orientation
                  << 0.0 << 0.0 << 0.0 << 1.0 << 0.0 << 0.0 << arma::endr // x vel 
                  << 0.0 << 0.0 << 0.0 << 0.0 << 1.0 << 0.0 << arma::endr // y vel 
                  << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 1.0 << arma::endr;// rotational vel
                                
         pbl::Matrix Fdim;
         Fdim << 1.0 << 0.0 << arma::endr               // width
              << 0.0 << 1.0 << arma::endr;              // length
                             
         // dim propagation -> No pos correction required as it is a constant dimension model.
         pbl::Matrix Pdim = rectangle_.get_Pdim();
         pbl::Matrix Q_k_dim = Q_k.submat(RECTANGLE_STATE_SIZE, RECTANGLE_STATE_SIZE, RECTANGLE_STATE_SIZE + RECTANGLE_DIM_STATE_SIZE -1, RECTANGLE_STATE_SIZE + RECTANGLE_DIM_STATE_SIZE -1);
         pbl::Vector x_k_1_k_1_dim = { rectangle_.get_w(), rectangle_.get_d()};
         pbl::Vector x_k_k_1_dim =  kalmanPropagate(Fdim, &Pdim, x_k_1_k_1_dim, Q_k_dim);
         
         if(Pdim(RM.width_dimRef, RM.width_dimRef) > MAX_DIMENSION_UNCERTAINTY)
         {
                 Pdim(RM.width_dimRef, RM.width_dimRef) = MAX_DIMENSION_UNCERTAINTY;
         }
         
         if(Pdim(RM.depth_dimRef, RM.depth_dimRef) > MAX_DIMENSION_UNCERTAINTY)
         {
                 Pdim(RM.depth_dimRef, RM.depth_dimRef) = MAX_DIMENSION_UNCERTAINTY;
         }
         
         rectangle_.set_w ( x_k_k_1_dim ( RM.width_dimRef ) );
         rectangle_.set_d ( x_k_k_1_dim ( RM.depth_dimRef ) );
         rectangle_.set_Pdim( Pdim );
         
         // Pos propagation
         pbl::Matrix P_PosVel = rectangle_.get_P_PosVel();
         pbl::Matrix Q_k_posVel = Q_k.submat(0, 0, RECTANGLE_STATE_SIZE - 1, RECTANGLE_STATE_SIZE - 1);
         
        pbl::Vector x_k_1_k_1_PosVel = {rectangle_.get_x(), rectangle_.get_y(), rectangle_.get_yaw(), rectangle_.get_xVel(), rectangle_.get_yVel(), rectangle_.get_yawVel() };
        pbl::Vector x_k_k_1_PosVel =  kalmanPropagate(F_PosVel, &P_PosVel, x_k_1_k_1_PosVel, Q_k_posVel);
        posVelState2Rectangle( x_k_k_1_PosVel );
        rectangle_.set_P_PosVel ( P_PosVel );
}

void FeatureProperties::updateRectangleFeatures ( pbl::Matrix R_k, pbl::Vector z_k)
{       
        unwrap( &z_k( RM.yaw_zRef ), (double) rectangle_.get_yaw(), (double) M_PI );

        if( rectangle_.switchDimensions( z_k( RM.yaw_zRef ) ) )
        {
                rectangle_.interchangeRectangleFeatures( );
                unwrap( &z_k( RM.yaw_zRef ), (double) rectangle_.get_yaw(), (double) M_PI ); 
        }
                         
       pbl::Matrix Pdim = rectangle_.get_Pdim();
        
       pbl::Vector x_k_k_1_dim = { rectangle_.get_w(), rectangle_.get_d()};
       pbl::Vector z_k_dim = { z_k( RM.width_zRef ), z_k( RM.depth_zRef ) };

       pbl::Matrix R_k_dim = R_k.submat(RECTANGLE_STATE_SIZE,
                                        RECTANGLE_STATE_SIZE, 
                                        RECTANGLE_STATE_SIZE + RECTANGLE_MEASURED_DIM_STATE_SIZE - 1, 
                                        RECTANGLE_STATE_SIZE + RECTANGLE_MEASURED_DIM_STATE_SIZE - 1);           
               
       bool arbitraryWidthObserved = (R_k_dim(RM.width_dimRef, RM.width_dimRef) > 0.75); // TODO better way of communicating this info 
       bool arbitraryDepthObserved = (R_k_dim(RM.depth_dimRef, RM.depth_dimRef) > 0.75 ); // TODO better way of communicating this info 
       
       pbl::Matrix H_dimensions = rectangle_.get_H_dim();

       if(arbitraryWidthObserved)
       {
               H_dimensions.at(RM.width_dimRef, RM.width_dimRef) = 0;
       }
       
       if(arbitraryDepthObserved)
       {
               H_dimensions.at(RM.depth_dimRef, RM.depth_dimRef) = 0;
       }
       
       // dim update
       pbl::Vector x_k_k_dim = kalmanUpdate(H_dimensions, &Pdim, x_k_k_1_dim, z_k_dim, R_k_dim);
       
       float deltaWidth = x_k_k_1_dim ( RM.width_dimRef ) - x_k_k_dim( RM.width_dimRef );
       float deltaDepth = x_k_k_1_dim ( RM.depth_dimRef ) - x_k_k_dim( RM.depth_dimRef );

       float deltaX = 0.0, deltaY = 0.0;
       float thetaPred = rectangle_.get_yaw();
       correctForDimensions( deltaWidth, deltaDepth, &deltaX, &deltaY, z_k( RM.x_zRef ), z_k( RM.y_zRef ), rectangle_.get_x(), rectangle_.get_y(), thetaPred );

       // Correct position due to updated dimensions
       rectangle_.set_x ( rectangle_.get_x() + deltaX );
       rectangle_.set_y ( rectangle_.get_y() + deltaY );

       rectangle_.set_w ( x_k_k_dim ( RM.width_dimRef ) );
       rectangle_.set_d ( x_k_k_dim ( RM.depth_dimRef ) );
        
        // Correct measured position caused by differences in modelled and measured dimensions
        deltaWidth = z_k ( RM.width_zRef ) - rectangle_.get_w();
        deltaDepth = z_k ( RM.depth_zRef ) - rectangle_.get_d();
        
        deltaX = 0.0; deltaY = 0.0;
        correctForDimensions( deltaWidth, deltaDepth, &deltaX, &deltaY, z_k( RM.x_zRef ), z_k( RM.y_zRef ), rectangle_.get_x(), rectangle_.get_y(), thetaPred );

        z_k( RM.x_zRef ) = z_k( RM.x_zRef ) - deltaX;
        z_k( RM.y_zRef ) = z_k( RM.y_zRef ) - deltaY;
        
        pbl::Matrix P_PosVel = rectangle_.get_P_PosVel();
        pbl::Vector x_k_k_1_PosVel = {rectangle_.get_x(), rectangle_.get_y(), rectangle_.get_yaw(), rectangle_.get_xVel(), rectangle_.get_yVel(), rectangle_.get_yawVel() };
        pbl::Vector z_k_posVel = {z_k ( RM.x_zRef ),     z_k ( RM.y_zRef ),     z_k ( RM.yaw_zRef )};
        pbl::Matrix R_k_posVel = R_k.submat(0, 0, RECTANGLE_MEASURED_STATE_SIZE - 1, RECTANGLE_MEASURED_STATE_SIZE - 1);
        pbl::Vector x_k_k_PosVel = kalmanUpdate(rectangle_.get_H_PosVel(), &P_PosVel, x_k_k_1_PosVel, z_k_posVel, R_k_posVel);
        posVelState2Rectangle( x_k_k_PosVel );

        rectangle_.set_P_PosVel ( P_PosVel );
        rectangle_.set_Pdim( Pdim );
}

void FeatureProperties::posVelState2Rectangle( pbl::Vector x_k_k_PosVel )
{
        rectangle_.set_x ( x_k_k_PosVel ( RM.x_PosVelRef ) );
        rectangle_.set_y ( x_k_k_PosVel ( RM.y_PosVelRef ) );
        rectangle_.set_yaw ( x_k_k_PosVel ( RM.yaw_PosVelRef ) );
        rectangle_.set_xVel ( x_k_k_PosVel ( RM.xVel_PosVelRef ) );
        rectangle_.set_yVel ( x_k_k_PosVel ( RM.yVel_PosVelRef ) );
        rectangle_.set_yawVel ( x_k_k_PosVel ( RM.yawVel_PosVelRef ) );
}

pbl::Gaussian Rectangle::rectangle2PDF(  )
{
        pbl::Gaussian G(RECTANGLE_STATE_SIZE + RECTANGLE_DIM_STATE_SIZE);
        G.setMean( getState() );
        G.setCovariance( getCovariance() );
        
        return G;
}

pbl::Gaussian Rectangle::observedRectangle2PDF(  )
{        
        pbl::Gaussian G_observed(RECTANGLE_MEASURED_STATE_SIZE + RECTANGLE_MEASURED_DIM_STATE_SIZE);
        G_observed.setMean( H_*getState() );
        G_observed.setCovariance( H_*getCovariance()*H_.t() );
        
        return G_observed;
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

void FeatureProperties::propagateCircleFeatures (pbl::Matrix Q_k, float dt)
{ 
         pbl::Matrix F_PosVel;
         F_PosVel << 1.0 << 0.0 << dt  << 0.0 << arma::endr  // x 
                  << 0.0 << 1.0 << 0.0 << dt  << arma::endr   // y 
                  << 0.0 << 0.0 << 1.0 << 0.0 << arma::endr  // x vel 
                  << 0.0 << 0.0 << 0.0 << 1.0 << arma::endr;  // y vel
                                 
         pbl::Matrix Fdim;
         Fdim << 1.0 << arma::endr;               // radius
                             
         // dim propagation -> No pos correction required as it is a constant dimension model.
         pbl::Matrix Pdim = circle_.get_Pdim();
         pbl::Matrix Q_k_dim = Q_k.submat(CIRCLE_STATE_SIZE, CIRCLE_STATE_SIZE, CIRCLE_STATE_SIZE + CIRCLE_DIM_STATE_SIZE -1, CIRCLE_STATE_SIZE + CIRCLE_DIM_STATE_SIZE -1);
         pbl::Vector x_k_1_k_1_dim = { circle_.get_radius()};
         pbl::Vector x_k_k_1_dim =  kalmanPropagate(Fdim, &Pdim, x_k_1_k_1_dim, Q_k_dim);
        
         circle_.set_radius ( x_k_k_1_dim ( CM.r_dimRef ) );
         circle_.set_Pdim( Pdim );
         
         // Pos propagation
         pbl::Matrix P_PosVel = circle_.get_P_PosVel();
         pbl::Matrix Q_k_posVel = Q_k.submat(0, 0, CIRCLE_STATE_SIZE - 1, CIRCLE_STATE_SIZE - 1);
         pbl::Vector x_k_1_k_1_PosVel = {circle_.get_x(), circle_.get_y(), circle_.get_xVel(), circle_.get_yVel() };
         pbl::Vector x_k_k_1_PosVel =  kalmanPropagate(F_PosVel, &P_PosVel, x_k_1_k_1_PosVel, Q_k_posVel);
        
         posVelState2Circle(x_k_k_1_PosVel);
         circle_.set_P_PosVel ( P_PosVel );
} 

void FeatureProperties::updateCircleFeatures ( pbl::Matrix R_k, pbl::Vector z_k )
// z = observation, dt is the time difference between the latest update and the new measurement
{       
        // First, update the dimensions
        pbl::Vector x_k_k_1_dim = { circle_.get_radius() };
        pbl::Vector z_k_dim = { z_k( CM.radius_zRef )};
        pbl::Matrix Pdim = circle_.get_Pdim();
        pbl::Matrix R_k_dim = R_k.submat(CIRCLE_STATE_SIZE,
                                         CIRCLE_STATE_SIZE,
                                         CIRCLE_STATE_SIZE + CIRCLE_MEASURED_DIM_STATE_SIZE - 1,
                                         CIRCLE_STATE_SIZE + CIRCLE_MEASURED_DIM_STATE_SIZE - 1);

        pbl::Vector x_k_k_dim = kalmanUpdate(circle_.get_H_dim(), &Pdim, x_k_k_1_dim, z_k_dim, R_k_dim);
        
        // After the position update for changed dimensions, update the dimensions
        pbl::Matrix P_PosVel = circle_.get_P_PosVel();
        pbl::Vector x_k_k_1_PosVel = {circle_.get_x(), circle_.get_y(), circle_.get_xVel(), circle_.get_yVel() };//, circle_.get_xAccel(), circle_.get_yAccel();
        pbl::Vector z_k_posVel = { z_k ( CM.x_zRef ), z_k ( CM.y_zRef ) };
        pbl::Matrix R_k_posVel = R_k.submat(0, 0, CIRCLE_MEASURED_STATE_SIZE - 1, CIRCLE_MEASURED_STATE_SIZE - 1);
         
        pbl::Vector x_k_k_PosVel = kalmanUpdate(circle_.get_H_PosVel(), &P_PosVel, x_k_k_1_PosVel, z_k_posVel, R_k_posVel);
         
        bool test = false;
        for(int iTest = 0; iTest<x_k_k_PosVel.size(); iTest++)
        {
                if(x_k_k_PosVel(iTest) != x_k_k_PosVel(iTest))
                {
                        test = true;
                        continue;
                }
        }
        
        posVelState2Circle(x_k_k_PosVel);
        
        circle_.set_radius ( x_k_k_dim( CM.r_dimRef ) );
        circle_.set_P_PosVel ( P_PosVel );
        circle_.set_Pdim ( Pdim );
}

void FeatureProperties::posVelState2Circle( pbl::Vector x_k_k_PosVel )
{       
        circle_.set_x ( x_k_k_PosVel ( CM.x_PosVelRef ) );
        circle_.set_y ( x_k_k_PosVel ( CM.y_PosVelRef ) );
        circle_.set_xVel ( x_k_k_PosVel ( CM.xVel_PosVelRef ) );
        circle_.set_yVel ( x_k_k_PosVel ( CM.yVel_PosVelRef ) );    
}

std::shared_ptr<pbl::Hybrid> FeatureProperties::getPDF()
{
           std::shared_ptr<pbl::Gaussian> rectPDF = std::make_shared<pbl::Gaussian>( rectangle_.rectangle2PDF() );
           std::shared_ptr<pbl::Gaussian> circPDF = std::make_shared<pbl::Gaussian>( circle_.circle2PDF() );
           
           // TODO initialize
           if(!observedProperties_)
           {
                   observedProperties_ = std::make_shared<pbl::Hybrid>();
           } else {
                   observedProperties_->clear();
           }
           
           observedProperties_->addPDF(*rectPDF, featureProbabilities_.get_pRectangle());
           observedProperties_->addPDF(*circPDF, featureProbabilities_.get_pCircle());
           
           return observedProperties_;
}

std::shared_ptr<pbl::Hybrid> FeatureProperties::getPDFSmall()
{        
           std::shared_ptr<pbl::Gaussian> rectPDF = std::make_shared<pbl::Gaussian>( rectangle_.observedRectangle2PDF() );
           std::shared_ptr<pbl::Gaussian> circPDF = std::make_shared<pbl::Gaussian>( circle_.observedCircle2PDF() );
           
           if(!observedProperties_)
           {
                   observedProperties_ = std::make_shared<pbl::Hybrid>();
           } else {
                   observedProperties_->clear();
           }
           
           observedProperties_->addPDF(*rectPDF, featureProbabilities_.get_pRectangle());
           observedProperties_->addPDF(*circPDF, featureProbabilities_.get_pCircle());
           
           return observedProperties_;
}

void FeatureProperties::printProperties()
{
        std::cout << "\t";
        rectangle_.printProperties();
        circle_.printProperties();
        std::cout << 
        "Probability circle = "    << featureProbabilities_.get_pCircle() << 
        "\t Probability rectangle = " << featureProbabilities_.get_pRectangle() << std::endl;
}

bool FeatureProperties::isValid()
{
    if(!rectangle_.isValid() || circle_.isValid() )
    {
            return false;
    } else {
            return true;
    }

}

}