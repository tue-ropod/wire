#include "wire_state_estimators/featureProperties.h"
#include <boost/iterator/iterator_concepts.hpp>


struct rectangleMapping {
   unsigned int x_PosVelRef = 0, y_PosVelRef = 1, yaw_PosVelRef = 2, xVel_PosVelRef = 3, yVel_PosVelRef = 4, yawVel_PosVelRef = 5;
   unsigned int width_dimRef = 0, depth_dimRef = 1;
   unsigned int x_zRef = 0, y_zRef = 1, yaw_zRef = 2, width_zRef = 3, depth_zRef = 4;
} RM;

struct circleMapping {
        unsigned int x_PosVelRef = 0, y_PosVelRef = 1, xVel_PosVelRef = 2, yVel_PosVelRef = 3;//, xAccel_PosVelRef = 4, yAccel_PosVelRef = 5;
        unsigned int r_dimRef = 0;      
        unsigned int x_zRef = 0, y_zRef = 1, radius_zRef = 2;
} CM;

namespace tracking{
        
Circle::Circle():   H_PosVel_( arma::eye(CIRCLE_MEASURED_STATE_SIZE, CIRCLE_STATE_SIZE) ), 
                    H_dim_ (arma::eye(CIRCLE_MEASURED_DIM_STATE_SIZE, CIRCLE_DIM_STATE_SIZE) )
{
    float notANumber = 0.0/0.0;
//     P_.setIdentity( 7, 7 );
    P_PosVel_ = arma::eye( CIRCLE_STATE_SIZE, CIRCLE_STATE_SIZE );
    Pdim_= arma::eye( CIRCLE_DIM_STATE_SIZE, CIRCLE_DIM_STATE_SIZE ); 
    this->setProperties( notANumber, notANumber, notANumber, 0.0, 0.0, notANumber, notANumber, notANumber, notANumber ); // Produces NaN values, meaning that the properties are not initialized yet
//     xVel_   = 0.0;
//     yVel_   = 0.0;
    
   // H_PosVel_ = arma::eye(CIRCLE_MEASURED_STATE_SIZE, CIRCLE_STATE_SIZE);
 //   H_dim_ = arma::eye(CIRCLE_MEASURED_DIM_STATE_SIZE, CIRCLE_DIM_STATE_SIZE);
    
//     pbl::Matrix zeroMatrix(H_PosVel_.n_rows,H_dim_.n_cols);
//     zeroMatrix.zeros(//     H_ = arma::join_cols(arma::join_rows(H_PosVel_,zeroMatrix),arma::join_rows(zeroMatrix.t(), H_dim_)); 
    
    
  /*  H_PosVel_ = arma::eye(CIRCLE_STATE_SIZE, CIRCLE_STATE_SIZE);
    for(unsigned int i = CIRCLE_MEASURED_STATE_SIZE; i < CIRCLE_STATE_SIZE; i++)
    {
            H_PosVel_(i,i) = 0.0;
    }
    */
    pbl::Matrix zeroMatrix(H_PosVel_.n_rows,H_dim_.n_cols);
     pbl::Matrix zeroMatrix2(H_dim_.n_rows, H_PosVel_.n_cols);
    zeroMatrix.zeros();
    zeroMatrix2.zeros();
 //   std::cout << "circle constructor: zeroMatrix = " << zeroMatrix << std::endl;
 //   std::cout << "circle constructor: zeroMatrix2 = " << zeroMatrix2 << std::endl;
    H_ = arma::join_cols(arma::join_rows(H_PosVel_,zeroMatrix),arma::join_rows(zeroMatrix2, H_dim_));     
    
 //   std::cout << "Cirlce construced: H_PosVel_ = " << H_PosVel_ << std::endl;
 //  std::cout << "Cirlce construced: H_dim_ = " << H_dim_ << std::endl;
 //   std::cout << "Cirlce construced: H_ = " << H_ << std::endl;
}

void Circle::setCircle( std::shared_ptr<const pbl::Gaussian> Gmeasured)
{       
     //   if (P->type() == pbl::PDF::GAUSSIAN) 
    //    {
         //       std::shared_ptr<const pbl::Gaussian> G = pbl::PDFtoGaussian(G);

//         std::cout << "setCircle: G = " << Gmeasured->toString() << std::endl;
        
        
           //     P_PosVel_ = G->getCovariance().submat(0, 0, CIRCLE_STATE_SIZE - 1, CIRCLE_STATE_SIZE - 1);
           //     Pdim_ = G->getCovariance().submat(CIRCLE_STATE_SIZE, CIRCLE_STATE_SIZE, CIRCLE_STATE_SIZE + CIRCLE_DIM_STATE_SIZE - 1, CIRCLE_STATE_SIZE + CIRCLE_DIM_STATE_SIZE - 1);

                this->setProperties( Gmeasured->getMean().at(CM.x_zRef),  // x
                                Gmeasured->getMean().at(CM.y_zRef), // y
                                0.3, // z TODO TEMP
                                0.0, //G->getMean().at(CM.xVel_PosVelRef), // xvel
                                0.0, //G->getMean().at(CM.yVel_PosVelRef), // yvel
                                0.0, // roll
                                0.0, // pitch
                                0.0, // yaw
                                Gmeasured->getMean().at(CM.radius_zRef)  ); // radius
           
            P_PosVel_(CM.x_PosVelRef, CM.x_PosVelRef) = Gmeasured->getCovariance()(CM.x_PosVelRef, CM.x_PosVelRef);
            P_PosVel_(CM.y_PosVelRef, CM.y_PosVelRef) = Gmeasured->getCovariance()(CM.y_PosVelRef, CM.y_PosVelRef);
            Pdim_(CM.r_dimRef, CM.r_dimRef) = Gmeasured->getCovariance()(CIRCLE_MEASURED_STATE_SIZE - 1 + CM.r_dimRef, CIRCLE_MEASURED_STATE_SIZE - 1 + CM.r_dimRef);
           
                
//                 std::cout << "setCircle P_PosVel_ = " << P_PosVel_ << std::endl;
//                 std::cout << "setCircle Pdim_ = " << Pdim_ << std::endl;
                
//                 std::cout << "setCircle, state = " << this->getState() << std::endl;
    //   }
    //    else {
     //            std::printf("Circle can only be set with Gaussians.\n");
     //   }
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
//     std::cout << "Pdim_ = " << Pdim_ << std::endl;
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

    marker.lifetime = ros::Duration ( TIMEOUT_TIME );
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
    
    marker.lifetime = ros::Duration ( TIMEOUT_TIME );
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
       
   
    
   /* for(unsigned int i = RECTANGLE_MEASURED_STATE_SIZE; i < RECTANGLE_STATE_SIZE; i++)
    {
            H_PosVel_(i,i) = 0.0;
    }*/

  //  H_dim_ =   arma::eye( RECTANGLE_MEASURED_DIM_STATE_SIZE, RECTANGLE_MEASURED_STATE_SIZE  );
    
 /*   pbl::Matrix zeroMatrix(H_PosVel_.n_rows,H_dim_.n_cols );
    zeroMatrix.zeros();
    H_ = arma::join_cols(arma::join_rows(H_PosVel_,zeroMatrix),arma::join_rows(zeroMatrix.t(),H_dim_));  
   */ 
    
    pbl::Matrix zeroMatrix(H_PosVel_.n_rows,H_dim_.n_cols);
    pbl::Matrix zeroMatrix2(H_dim_.n_rows, H_PosVel_.n_cols);
    
    zeroMatrix.zeros();
    zeroMatrix2.zeros();
  /*  std::cout << "Rectangle constructor: zeroMatrix = " << zeroMatrix << std::endl;
    std::cout << "Rectangle constructor: zeroMatrix2 = " << zeroMatrix2 << std::endl;
    */
    H_ = arma::join_cols(arma::join_rows(H_PosVel_,zeroMatrix),arma::join_rows(zeroMatrix2, H_dim_));    
    
/*    std::cout << "Rectangle construced: H_dim_ = " << H_dim_ << std::endl;
    std::cout << "Rectangle construced: H_PosVel_ = " << H_PosVel_ << std::endl;
    std::cout << "Rectangle construced: H_PosVel_ = " << H_ << std::endl;
    */
}


void Rectangle::setRectangle( std::shared_ptr<const pbl::Gaussian> Gmeasured)
{
  // if (P->type() == pbl::PDF::GAUSSIAN) 
 //  {
    //       std::shared_ptr<const pbl::Gaussian> G = pbl::PDFtoGaussian(G);
        
//         std::cout << "setRectangle G = " << Gmeasured->toString() << std::endl;
//         std::cout << "G->getCovariance = " << Gmeasured->getCovariance() << std::endl;
//         std::cout << "G->getMean().size() = " << Gmeasured->getMean().size() << std::endl;
        
          /* P_PosVel_ = G->getCovariance().submat(0, 0, RECTANGLE_MEASURED_STATE_SIZE - 1, RECTANGLE_MEASURED_STATE_SIZE - 1);
           Pdim_ = G->getCovariance().submat(RECTANGLE_MEASURED_STATE_SIZE, 
                                             RECTANGLE_MEASURED_STATE_SIZE, 
                                             RECTANGLE_MEASURED_STATE_SIZE + RECTANGLE_MEASURED_DIM_STATE_SIZE - 1, 
                                             RECTANGLE_MEASURED_STATE_SIZE + RECTANGLE_MEASURED_DIM_STATE_SIZE - 1);
            */    
          
           this->setValues( Gmeasured->getMean().at(RM.x_zRef),  // x
                            Gmeasured->getMean().at(RM.y_zRef), // y
                            0.3, // z TODO TEMP
                            Gmeasured->getMean().at(RM.width_zRef), // w
                            Gmeasured->getMean().at(RM.depth_zRef), // d
                            0.1, // h TODO TEMP
                            0.0, // roll
                            0.0, // pitch
                            Gmeasured->getMean().at(RM.yaw_zRef) ); // yaw
           
            P_PosVel_(RM.x_PosVelRef, RM.x_PosVelRef) = Gmeasured->getCovariance()(RM.x_PosVelRef, RM.x_PosVelRef);
            P_PosVel_(RM.y_PosVelRef, RM.y_PosVelRef) = Gmeasured->getCovariance()(RM.y_PosVelRef, RM.y_PosVelRef);
            P_PosVel_(RM.yaw_PosVelRef, RM.yaw_PosVelRef) = Gmeasured->getCovariance()(RM.yaw_PosVelRef, RM.yaw_PosVelRef);
            Pdim_(RM.width_dimRef, RM.width_dimRef) = Gmeasured->getCovariance()(RECTANGLE_MEASURED_STATE_SIZE - 1 + RM.width_dimRef, RECTANGLE_MEASURED_STATE_SIZE - 1 +RM.width_dimRef);
            Pdim_(RM.depth_dimRef, RM.depth_dimRef) = Gmeasured->getCovariance()(RECTANGLE_MEASURED_STATE_SIZE - 1 + RM.depth_dimRef, RECTANGLE_MEASURED_STATE_SIZE - 1 + RM.depth_dimRef);
           
        /*    xVel_   = G->getMean().at(RM.xVel_PosVelRef);
            yVel_   = G->getMean().at(RM.yVel_PosVelRef);
            yawVel_ = G->getMean().at(RM.yawVel_PosVelRef); 
          */  
            
//                 std::cout << "setRectangle P_PosVel_ = " << P_PosVel_ << std::endl;
//                 std::cout << "setRectangle Pdim_ = " << Pdim_ << std::endl;
                
//                 std::cout << "setRectangle, state = " << this->getState() << std::endl;
  //   }
  //   else {
  //               std::printf("Rectangle can only be set with Gaussians.\n");
  //   }
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
    marker.lifetime = ros::Duration ( TIMEOUT_TIME );
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
    
    marker.lifetime = ros::Duration ( TIMEOUT_TIME );
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
    
    marker.lifetime = ros::Duration ( TIMEOUT_TIME );
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

/*void Rectangle::setState(float posX, float posY, float posYaw, float xVel, float yVel, float yawVel, float width, float depth)
{
         pbl::Vector state( 8, 1 );
         state << posX, posY, posYaw, xVel, yVel, yawVel, width, depth;
         
         //return state;
}*/

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
        // TODO improve
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
        if(pCircle < MIN_PROB_OBJECT) // smooth out prob such that recovery is easier
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

    pmf_measured->setProbability ( "Rectangle", pRectangle_measured );
    pmf_measured->setProbability ( "Circle", pCircle_measured );

    pmf_->update ( pmf_measured );
    
    float pCircle = pmf_->getProbability ( "Circle" );            
    
    if(pCircle < MIN_PROB_OBJECT) // smooth out prob such that recovery is easier
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
}

void FeatureProbabilities::update ( FeatureProbabilities& featureProbabilities_in )
{
    pmf_->update ( featureProbabilities_in.pmf_ );
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

pbl::Vector kalmanPropagate(pbl::Matrix F, pbl::Matrix *P, pbl::Vector x_k_1_k_1, pbl::Matrix Q)
{   
    pbl::Vector x_k_k_1 = F*x_k_1_k_1;
    pbl::Matrix P_k_k_1 = F* (*P) * F.t() + Q;
    
    return x_k_k_1;
}
    
pbl::Vector kalmanUpdate(pbl::Matrix H, pbl::Matrix *P, pbl::Vector x_k_k_1, pbl::Vector z_k, pbl::Matrix R)
{   
    pbl::Matrix I = arma::eye(P->n_rows, P->n_cols);
    pbl::Vector y_k = z_k - H*x_k_k_1;
    pbl::Matrix S_k = H* *P*H.t() + R;
    
    
   /* std::cout << "kalmanUpdate: S_k = " << S_k << std::endl;
    std::cout << "kalmanUpdate: H = " << H << std::endl;
    std::cout << "kalmanUpdate: H.t() = " << H.t() << std::endl;
    std::cout << "kalmanUpdate: *P = " << *P << std::endl;*/
//     std::cout << "kalmanUpdate: R = " << R << std::endl;
    
    //std::cout << "kalmanUpdate: H* *P*H.t() = " << H* *P*H.t() << std::endl;

    pbl::Vector K_k = *P*H.t() * inv(S_k);
    pbl::Vector x_k_k = x_k_k_1 + K_k*y_k;
 //   pbl::Matrix P_k_k = ( I - K_k*H )* *P;  

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
        
         rectangle_.set_w ( x_k_k_1_dim ( RM.width_dimRef ) );
         rectangle_.set_d ( x_k_k_1_dim ( RM.depth_dimRef ) );
         rectangle_.set_Pdim( Pdim );
         
         // Pos propagation
         pbl::Matrix P_PosVel = rectangle_.get_P_PosVel();
         pbl::Matrix Q_k_posVel = Q_k.submat(0, 0, RECTANGLE_STATE_SIZE - 1, RECTANGLE_STATE_SIZE - 1);
        //pbl::Vector x_k_1_k_1_PosVel( 6, 1 ), z_k_posVel( 3, 1 );
        pbl::Vector x_k_1_k_1_PosVel = {rectangle_.get_x(), rectangle_.get_y(), rectangle_.get_yaw(), rectangle_.get_xVel(), rectangle_.get_yVel(), rectangle_.get_yawVel() };
        pbl::Vector x_k_k_1_PosVel =  kalmanPropagate(F_PosVel, &P_PosVel, x_k_1_k_1_PosVel, Q_k_posVel);
        
        posVelState2Rectangle( x_k_k_1_PosVel );
        rectangle_.set_P_PosVel ( P_PosVel );
}

//void FeatureProperties::updateRectangleFeatures (pbl::Matrix Q_k, pbl::Matrix R_k, pbl::Vector z_k, float dt)
void FeatureProperties::updateRectangleFeatures ( pbl::Matrix R_k, pbl::Vector z_k)
{
        // z = observation, dt is the time difference between the latest update and the new measurement
        // 2 stages: first determine the updated width en depth, then use this difference to update the position first in order to prevent ghost-velocities. 
        
        // conversion for general state to state for (1) the position and velocity state and (2) the dimension state
       

       // pbl::Matrix F_PosVel ( RECTANGLE_STATE_SIZE, RECTANGLE_STATE_SIZE );
     /*   pbl::Matrix F_PosVel = { { 1.0, 0.0, 0.0, dt,  0.0, 0.0}, // x 
                                 { 0.0, 1.0, 0.0, 0.0, dt,  0.0}, // y 
                                 { 0.0, 0.0, 1.0, 0.0, 0.0, dt}, // orientation
                                 { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0}, // x vel 
                                 { 0.0, 0.0, 0.0, 0.0, 1.0, 0.0}, // y vel 
                                 { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0} }; // rotational vel
       */
     
        //pbl::Matrix Fdim ( RECTANGLE_DIM_STATE_SIZE, RECTANGLE_DIM_STATE_SIZE );    
      /*  pbl::Matrix Fdim = { { 1.0, 0.0},               // width
                             { 0.0, 1.0} };               // length
                             */
                
       // Eigen::MatrixXf H_PosVel ( 3, 6 );
      /*  pbl::Matrix H_PosVel = { { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                               {0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
                               {0.0, 0.0, 1.0, 0.0, 0.0, 0.0} };
         
        pbl::Matrix Hdim = arma::eye( 2, 2 );
        */
        //Hdim.setIdentity( Hdim.rows(), Hdim.cols() ); 
                
        unwrap( &z_k( RM.yaw_zRef ), (double) rectangle_.get_yaw(), (double) M_PI );

        if( rectangle_.switchDimensions( z_k( RM.yaw_zRef ) ) )
        {
                rectangle_.interchangeRectangleFeatures( );
                unwrap( &z_k( RM.yaw_zRef ), (double) rectangle_.get_yaw(), (double) M_PI ); 
                
                //ROS_WARN("Interchanged rectangular features");
        }
        
        pbl::Matrix Pdim = rectangle_.get_Pdim();
       // pbl::Matrix x_k_1_k_1_dim( 2, 1 ), z_k_dim( 2, 1 );
        pbl::Vector x_k_k_1_dim = { rectangle_.get_w(), rectangle_.get_d()};
        pbl::Vector z_k_dim = { z_k( RM.width_zRef ), z_k( RM.depth_zRef ) };
        // pbl::Vector x_k_k_dim = kalmanUpdate(Fdim, rectangle_.H_dim, &Pdim, x_k_1_k_1_dim, z_k_dim, Q_k.block<2, 2>( 6, 6 ), R_k.block<2, 2>( 3, 3 ) );
       pbl::Matrix R_k_dim = R_k.submat(RECTANGLE_MEASURED_STATE_SIZE, RECTANGLE_MEASURED_STATE_SIZE, RECTANGLE_MEASURED_STATE_SIZE + RECTANGLE_MEASURED_DIM_STATE_SIZE - 1, RECTANGLE_MEASURED_STATE_SIZE + RECTANGLE_MEASURED_DIM_STATE_SIZE - 1);        
        
//        std::cout << "R_k_dim = " << R_k_dim << std::endl;
       
       // dim update
//        std::cout << "Rectangle: dim update of Kalman" << std::endl;
       pbl::Vector x_k_k_dim = kalmanUpdate(rectangle_.get_H_dim(), &Pdim, x_k_k_1_dim, z_k_dim, R_k_dim);
        
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
        //pbl::Vector x_k_1_k_1_PosVel( 6, 1 ), z_k_posVel( 3, 1 );
        pbl::Vector x_k_k_1_PosVel = {rectangle_.get_x(), rectangle_.get_y(), rectangle_.get_yaw(), rectangle_.get_xVel(), rectangle_.get_yVel(), rectangle_.get_yawVel() };
        pbl::Vector z_k_posVel = {z_k ( RM.x_zRef ),     z_k ( RM.y_zRef ),     z_k ( RM.yaw_zRef )};
        pbl::Matrix R_k_posVel = R_k.submat(0, 0, RECTANGLE_MEASURED_STATE_SIZE - 1, RECTANGLE_MEASURED_STATE_SIZE - 1);
        
//         std::cout << "R_k_posVel = " << R_k_posVel << std::endl;
        
//         std::cout << "Rectangle: pos vel update of Kalman" << std::endl;
        pbl::Vector x_k_k_PosVel = kalmanUpdate(rectangle_.get_H_PosVel(), &P_PosVel, x_k_k_1_PosVel, z_k_posVel, R_k_posVel);
 
        posVelState2Rectangle( x_k_k_PosVel );

        rectangle_.set_P_PosVel ( P_PosVel );
        rectangle_.set_Pdim( Pdim );
        
//         std::cout << "End of Kalman update: values = " << std::endl;
//         rectangle_.printProperties();
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
        pbl::Gaussian G_observed(RECTANGLE_STATE_SIZE + RECTANGLE_DIM_STATE_SIZE);
//         std::cout << "observedRectangle2PDF: state = " << getState() << std::endl;
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
        pbl::Gaussian G_observed(CIRCLE_STATE_SIZE + CIRCLE_DIM_STATE_SIZE);
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
         circle_.set_radius ( x_k_k_1_PosVel( CM.r_dimRef ) );
         circle_.set_P_PosVel ( P_PosVel );
} 

void FeatureProperties::updateCircleFeatures ( pbl::Matrix R_k, pbl::Vector z_k )
// z = observation, dt is the time difference between the latest update and the new measurement
{       
        //Eigen::MatrixXf F_PosVel ( 4, 4 );
        /*pbl::Matrix F_PosVel =  { {1.0, 0.0, dt,  0.0},  // x 
                                  { 0.0, 1.0, 0.0, dt},   // y 
                                  { 0.0, 0.0, 1.0, 0.0},  // x vel 
                                  { 0.0, 0.0, 0.0, 1.0} };  // y vel
        */
                                  
        //float dt2 = std::pow(dt, 2.0);
                    
        //Eigen::MatrixXf Fdim ( 1, 1 );    
        /*pbl::Matrix Fdim = { {1.0} };               // radius*/
                    
        //Eigen::MatrixXf H_PosVel ( 2, 4 );
        /*pbl::Matrix H_PosVel = { {1.0, 0.0, 0.0, 0.0 },
                                 { 0.0, 1.0, 0.0, 0.0 } };
         */
        /*
        Eigen::MatrixXf Hdim ( 1, 1 );
        pbl::Matrix Hdim = { {1.0} };;
        */
        
//         std::cout << "updateCircleFeatures: " << std::endl;
        
//         std::cout << "R_k = " << R_k << std::endl;
        
        // First, update the dimensions
        //p x_k_1_k_1_dim ( 1, 1 ), z_k_dim( 1, 1 );
        pbl::Vector x_k_k_1_dim = { circle_.get_radius() };
        pbl::Vector z_k_dim = { z_k( CM.radius_zRef )};
        pbl::Matrix Pdim = circle_.get_Pdim();
        pbl::Matrix R_k_dim = R_k.submat(CIRCLE_MEASURED_STATE_SIZE, CIRCLE_MEASURED_STATE_SIZE, CIRCLE_MEASURED_STATE_SIZE + CIRCLE_MEASURED_DIM_STATE_SIZE - 1, CIRCLE_MEASURED_STATE_SIZE + CIRCLE_MEASURED_DIM_STATE_SIZE - 1);        
//         std::cout << "R_k_dim = " << R_k_dim << std::endl;
        
        //pbl::Vector x_k_k_dim = kalmanUpdate(Fdim, circle_.H_dim_, &Pdim, x_k_1_k_1_dim, z_k_dim, Q_k.block<1, 1>( 4, 4 ), R_k.block<1, 1>( 2, 2 ) );
        
        std::cout << "Circle update: x_k_k_1_dim = " << x_k_k_1_dim << std::endl;
        pbl::Vector x_k_k_dim = kalmanUpdate(circle_.get_H_dim(), &Pdim, x_k_k_1_dim, z_k_dim, R_k_dim);
        std::cout << "Circle update: x_k_k_dim = " << x_k_k_dim << std::endl;
        
        // After the position update for changed dimensions, update the dimensions
        pbl::Matrix P_PosVel = circle_.get_P_PosVel();
       // pbl::Matrix x_k_1_k_1_PosVel( 4, 1 ), z_k_posVel( 2, 1 );
        pbl::Matrix x_k_k_1_PosVel = {circle_.get_x(), circle_.get_y(), circle_.get_xVel(), circle_.get_yVel() };//, circle_.get_xAccel(), circle_.get_yAccel();
        std::cout << "Circle update: x_k_k_1_PosVel = " << x_k_k_1_dim << std::endl;
        pbl::Matrix z_k_posVel = { z_k ( CM.x_zRef ), z_k ( CM.y_zRef ) };
        //pbl::Matrix x_k_k_PosVel = kalmanUpdate(F_PosVel, circle_.H_PosVel, &P_PosVel, x_k_1_k_1_PosVel, z_k_posVel, Q_k.block<4, 4>( 0, 0 ), R_k.block<2, 2>( 0, 0 ) );
         pbl::Matrix R_k_posVel = R_k.submat(0, 0, CIRCLE_MEASURED_STATE_SIZE - 1, CIRCLE_MEASURED_STATE_SIZE - 1);
         pbl::Vector x_k_k_PosVel = kalmanUpdate(circle_.get_H_PosVel(), &P_PosVel, x_k_k_1_PosVel, z_k_posVel, R_k_posVel);
         std::cout << "Circle update: x_k_k_PosVel = " << x_k_k_PosVel << std::endl;
         
          bool test = false;
         for(int iTest = 0; iTest<x_k_k_PosVel.size(); iTest++)
         {
                 if(x_k_k_PosVel(iTest) != x_k_k_PosVel(iTest))
                 {
                         test = true;
                         continue;
                 }
         }
         // break featureProperties.cpp:998 if test > 0
         
//          std::cout << "R_k_posVel = " << R_k_posVel << std::endl;
         
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
//         std::cout << "FeatureProperties::getPDF()" << std::endl;
        
//         rectangle_.printProperties();
//         circle_.printProperties();
        
           std::shared_ptr<pbl::Gaussian> rectPDF = std::make_shared<pbl::Gaussian>( rectangle_.observedRectangle2PDF() );
           std::shared_ptr<pbl::Gaussian> circPDF = std::make_shared<pbl::Gaussian>( circle_.observedCircle2PDF() );
        //   std::shared_ptr<pbl::PMF> probPDF = std::make_shared<pbl::PMF>(*featureProbabilities_.pmf_);
           
//            std::cout << "rectPDF = " << rectPDF->toString() << std::endl;
//            std::cout << "circPDF = " << circPDF->toString() << std::endl;
      //     std::cout << "probPDF = " << probPDF->toString() << std::endl;
            
           // TODO initialize
           if(!observedProperties_)
           {
                   observedProperties_ = std::make_shared<pbl::Hybrid>();
           } else {
                   observedProperties_->clear();
           }
           
           observedProperties_->addPDF(*rectPDF, featureProbabilities_.get_pRectangle());
           observedProperties_->addPDF(*circPDF, featureProbabilities_.get_pCircle());
          // observedProperties_->addPDF(*probPDF, 1.0);
           
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

}