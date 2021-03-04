#include "featureProperties.h"
#include <boost/iterator/iterator_concepts.hpp>

namespace tracking{

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
        std::cout << "FeatureProperties::updateRectangleFeatures: measured = " << &z_k( RM.yaw_zRef) << " model = " << (double) rectangle_.get_yaw() << std::endl;
        unwrap( &z_k( RM.yaw_zRef ), (double) rectangle_.get_yaw(), (double) M_PI );

        std::cout << "FeatureProperties::updateRectangleFeatures: unwrapped = " << &z_k( RM.yaw_zRef) << " model = " << (double) rectangle_.get_yaw() << std::endl;

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

void FeatureProperties::correctForDimensions(   float deltaWidth, float deltaDepth, float* xMeasured, float* yMeasured, 
                                                float measuredPosX, float measuredPosY, float modelledPosX, float modelledPosY, 
                                                float thetaPred)
{
        float deltaX_Width, deltaY_Width, deltaX_Depth, deltaY_Depth;
        correctPosForDimDiff(deltaWidth, 0, &deltaX_Width, &deltaY_Width, thetaPred );
        correctPosForDimDiff(0, deltaDepth, &deltaX_Depth, &deltaY_Depth, thetaPred );
        
        float distPosWidth2 = pow( modelledPosX + deltaX_Width - measuredPosX, 2.0 ) + pow( modelledPosY + deltaY_Width - measuredPosY, 2.0 );
        float distNegWidth2 = pow( modelledPosX - deltaX_Width - measuredPosX, 2.0 ) + pow( modelledPosY - deltaY_Width - measuredPosY, 2.0 );
        
        float distPosDepth2 = pow( modelledPosX + deltaX_Depth - measuredPosX, 2.0 ) + pow( modelledPosY + deltaY_Depth - measuredPosY, 2.0 );
        float distNegDepth2 = pow( modelledPosX - deltaX_Depth - measuredPosX, 2.0 ) + pow( modelledPosY - deltaY_Depth - measuredPosY, 2.0 );
        
        // TODO what for single dim update 
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

void FeatureProperties::printProperties()
{
        std::cout << "\t";
        rectangle_.printProperties();
        circle_.printProperties();
        std::cout << 
        "Probability circle = "    << featureProbabilities_.get_pCircle() << 
        "\t Probability rectangle = " << featureProbabilities_.get_pRectangle() << std::endl;
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

bool FeatureProperties::isValid()
{ 
    if(!rectangle_.isValid() || circle_.isValid() ||
     featureProbabilities_.get_pRectangle() != featureProbabilities_.get_pRectangle() ||
     featureProbabilities_.get_pCircle()    != featureProbabilities_.get_pCircle() )
    {
            return false;
    } else {
            return true;
    }

}

}