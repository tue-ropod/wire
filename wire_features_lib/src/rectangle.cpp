#include "rectangle.h"

namespace tracking{

struct rectangleMapping RM;

Rectangle::Rectangle(): H_dim_ ( arma::eye(RECTANGLE_MEASURED_DIM_STATE_SIZE, RECTANGLE_DIM_STATE_SIZE)),
                        H_PosVel_( arma::eye(RECTANGLE_MEASURED_STATE_SIZE, RECTANGLE_STATE_SIZE) )
{
    float notANumber = 0.0/0.0;
    P_PosVel_ = arma::eye( RECTANGLE_STATE_SIZE, RECTANGLE_STATE_SIZE ); 
    Pdim_ = 2*arma::eye( RECTANGLE_DIM_STATE_SIZE, RECTANGLE_DIM_STATE_SIZE ); 
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
        Pdim_ = 2*arma::eye( RECTANGLE_DIM_STATE_SIZE, RECTANGLE_DIM_STATE_SIZE ); 
        
        P_PosVel_.submat(0, 0, RECTANGLE_MEASURED_STATE_SIZE - 1, RECTANGLE_MEASURED_STATE_SIZE - 1) = 
                Gmeasured->getCovariance().submat(0, 0, RECTANGLE_MEASURED_STATE_SIZE - 1, RECTANGLE_MEASURED_STATE_SIZE - 1);
        
        Pdim_.submat(0, 0, RECTANGLE_MEASURED_DIM_STATE_SIZE - 1, RECTANGLE_MEASURED_DIM_STATE_SIZE - 1) =
                 Gmeasured->getCovariance().submat(RECTANGLE_MEASURED_STATE_SIZE, 
                                                   RECTANGLE_MEASURED_STATE_SIZE,
                                                   RECTANGLE_MEASURED_STATE_SIZE + RECTANGLE_MEASURED_DIM_STATE_SIZE - 1,
                                                   RECTANGLE_MEASURED_STATE_SIZE + RECTANGLE_MEASURED_DIM_STATE_SIZE - 1);
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

bool Rectangle::switchDimensions( float measuredYaw)
{
        // wat als dit een veelvoud is?!
        std::cout << "switchDimensions: std::fabs( std::fabs( yaw_ - measuredYaw )- M_PI_2 ) = " << std::fabs( std::fabs( yaw_ - measuredYaw )- M_PI_2 ) << std::endl;
        return std::fabs( std::fabs( yaw_ - measuredYaw )- M_PI_2 ) < MARGIN_RECTANGLE_INTERCHANGE;
}

void Rectangle::interchangeRectangleFeatures()
{
        std::cout << " interchagne rectangle features" << std::endl;
        float widthOld = w_;
        w_ = d_;
        d_ = widthOld;
        
       yaw_ += M_PI_2;
       
       float P_depthOld = Pdim_ ( 1, 1 );
       Pdim_( 1, 1) = Pdim_( 0, 0);
       Pdim_( 0, 0) = P_depthOld;
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


}
