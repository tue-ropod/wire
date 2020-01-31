/*
 * MyObject.cpp
 *
 *  Created on: Apr 27, 2011
 *      Author: sdries
 */

#include "wire/storage/SemanticObject.h"
#include "wire/storage/KnowledgeDatabase.h"

#include "wire/core/Evidence.h"
#include "wire/core/ClassModel.h"
#include "wire/core/Property.h"
#include "wire/logic/Assignment.h"
#include "wire/logic/Hypothesis.h"

#include <problib/conversions.h>

using namespace std;

namespace mhf {
        
         // TODO single file for these standard functions
template <typename T> 
int sgn(T val) 
{
    return (T(0) < val) - (val < T(0));
}

template <typename T> 
bool unwrap (T *angleMeasured, T angleReference, T increment)
{
        bool updated = false;
        // Rectangle is symmetric over pi-radians, so unwrap to pi
        T diff = angleReference - *angleMeasured;
        
        int d = diff / (increment);
        *angleMeasured += d*increment;
        
        if( d!= 0)
        {
                updated = true;
        }
        
        T r = angleReference - *angleMeasured;
        
        if( fabs(r) > (0.5*increment) )
        {
                *angleMeasured += sgn(r)*increment;
                updated = true;
        }
        return updated;
}

int SemanticObject::N_SEMANTICOBJECT = 0;

SemanticObject::SemanticObject(long ID) : ID_(ID), expected_class_("") {
//         std::cout << "Constructor of semantic object" << std::endl;
    ++N_SEMANTICOBJECT;
//     std::cout << "N_SEMANTICOBJECT = " << N_SEMANTICOBJECT << std::endl;
    
    parent_hypotheses_ = new std::set<Hypothesis*>();
    
//     std::cout << "Semantic object constructor: parent_hypotheses_ = " << parent_hypotheses_ << " semantic object = " << this << std::endl;
}

SemanticObject::SemanticObject(const SemanticObject& orig)
    : PropertySet(orig), ID_(orig.ID_), expected_class_(orig.expected_class_) {
//             std::cout << "Copy constructor of semantic object" << std::endl;
    ++N_SEMANTICOBJECT;
//     std::cout << "N_SEMANTICOBJECT = " << N_SEMANTICOBJECT << std::endl;
       parent_hypotheses_ = new std::set<Hypothesis*>();
    
//     std::cout << "Semantic object copy constructor: parent_hypotheses_ = " << parent_hypotheses_ << " semantic object = " << this << std::endl;
}

SemanticObject::~SemanticObject() {
    --N_SEMANTICOBJECT;
    if(parent_hypotheses_)
    {
//             std::cout << "semantic object destructor: delete parent_hypothesis, semantic object = " << this << std::endl;
            delete parent_hypotheses_;
    }
}

void SemanticObject::init(const Evidence& z) {
    update(z);
}

double SemanticObject::getLikelihood(const PropertySet& ev) const {

    //propagate(ev.getTimeStamp());

    double likelihood = 1;

    vector<Attribute> need_to_deduce;

    const map<Attribute, std::shared_ptr<Property>>& ev_props = ev.getPropertyMap();
    for(map<Attribute, std::shared_ptr<Property>>::const_iterator it = ev_props.begin(); it != ev_props.end(); ++it) {
        const Attribute& attribute = it->first;
        std::shared_ptr<const Property> ev_prop = it->second;

        std::shared_ptr<const Property> this_prop = getProperty(attribute);

        if (this_prop) {
                
                std::shared_ptr<pbl::PDF> z = ev_prop->getValue()->clone();
                Attribute positionAndDimension = mhf::AttributeConv::attribute("positionAndDimension");

                if(attribute == positionAndDimension)
                {
                        std::shared_ptr<const pbl::Hybrid> modelledValueHyb = pbl::PDFtoHybrid( this_prop->getValue() );
                        std::vector<pbl::Hybrid::distributionStruct> modelledPDFs = modelledValueHyb->getPDFS();
                        std::shared_ptr<const pbl::Gaussian> modelledRectGauss = pbl::PDFtoGaussian( modelledPDFs[0].pdf );// TODO proper numbering for conversion
                        
                        std::shared_ptr<const pbl::Hybrid> measuredValueHyb = pbl::PDFtoHybrid(z);
                        std::vector<pbl::Hybrid::distributionStruct> measuredPDFs = measuredValueHyb->getPDFS();
                        std::shared_ptr< const pbl::PDF> measuredRectPDF = measuredPDFs[0].pdf; // TODO proper numbering for conversion
                        std::shared_ptr<const pbl::Gaussian> measuredRectGauss = pbl::PDFtoGaussian(measuredRectPDF);
                        
                        unsigned int yaw_zRef = 2; // TODO proper numbering for conversion
                        unsigned int yaw_PosVelRef = 2; // TODO proper numbering for conversion
                        arma::vec muMeasured = measuredRectGauss->getMean();
                         
                        if( unwrap( &(muMeasured.at(yaw_zRef)), (double)  modelledRectGauss->getMean().at(yaw_PosVelRef), (double) M_PI ) )      
                        {
                                std::cout << "SemanticObject.cpp: Updated yaw. muMeasured.at(yaw_zRef) = " << muMeasured.at(yaw_zRef) << 
                                " modelledRectGauss->getMean().at(yaw_PosVelRef) = " << modelledRectGauss->getMean().at(yaw_PosVelRef) << std::endl;
                                
                                std::shared_ptr<pbl::Gaussian> updatedRectGauss = std::make_shared<pbl::Gaussian>(muMeasured, measuredRectGauss->getCovariance());
                                std::shared_ptr< pbl::Hybrid> measuredValueHybUpdated = std::make_shared<pbl::Hybrid>();
                        
                                measuredValueHybUpdated->addPDF(*updatedRectGauss,modelledPDFs[0].weight);
                                measuredValueHybUpdated->addPDF(*modelledPDFs[1].pdf,modelledPDFs[1].weight);
                                
                                z = measuredValueHybUpdated;
                        }
                }
                
                std::cout << "SemanticObject::getLikelihood: z = " << z->toString() << std::endl;
                
             likelihood *= this_prop->getLikelihood(z);
        } else {
            need_to_deduce.push_back(attribute);
        }
    }

    vector<Property> deduced_props = KnowledgeDatabase::getInstance().inferProperties(*this, need_to_deduce);

    for(vector<Property>::iterator it_prop = deduced_props.begin(); it_prop != deduced_props.end(); ++it_prop) {
        std::shared_ptr<const Property> ev_prop = ev.getProperty(it_prop->getAttribute());
        assert(ev_prop);
        likelihood *= it_prop->getLikelihood(ev_prop->getValue());
    }

    // cout << "    Likelihood existing = " << likelihood << endl;

    return likelihood;
}

void SemanticObject::update(const Evidence& ev) {
// std::cout << "SemanticObject::update: going to propagate." << std::endl;
    propagate(ev.getTimestamp());

    // first update class property

    Attribute class_att = AttributeConv::attribute("class_label");
    std::shared_ptr<const Property> ev_class = ev.getProperty(class_att);

    if (ev_class) {
        std::shared_ptr<Property> my_class = getProperty(class_att);

        if (my_class) {
            my_class->update(ev_class->getValue(), ev.getTimestamp());
        } else {
            std::shared_ptr<const IStateEstimator> prototype = getExpectedObjectModel().getEstimator(class_att);
            if (prototype) {
                std::shared_ptr<Property> new_prop = std::make_shared<Property>(class_att, *prototype);
                new_prop->update(ev_class->getValue(), ev.getTimestamp());
                addProperty(new_prop);
            } else {
                printf("Could not find prototype estimator for attribute '%s'\n", AttributeConv::attribute_str(class_att).c_str());
            }
        }
    }

    bool class_changed = false;
    const ClassModel& class_model = getExpectedObjectModel();

    if (expected_class_ != class_model.getModelName()) {
        expected_class_ = class_model.getModelName();
        class_changed = true;
    }

    // then update rest

    for(map<Attribute, std::shared_ptr<Property>>::const_iterator it = ev.getPropertyMap().begin(); it != ev.getPropertyMap().end(); ++it) {

        const Attribute& attribute = it->first;
        std::shared_ptr<const Property> ev_prop = it->second;

        if (attribute != class_att) {
            std::shared_ptr<Property> my_prop = getProperty(attribute);

            if (my_prop && !class_changed) {
                my_prop->update(ev_prop->getValue(), ev.getTimestamp());
                //cout << "Updating " << AttributeConv::attribute_str(attribute) << " with " << ev_prop->getValue().toString() << endl;
                //cout << "Result: " << my_prop->toString() << endl;
            } else {
                std::shared_ptr<const IStateEstimator> prototype = getExpectedObjectModel().getEstimator(attribute);
                if (prototype) {
                    std::shared_ptr<Property> new_prop = std::make_shared<Property>(attribute, *prototype);
                    new_prop->update(ev_prop->getValue(), ev.getTimestamp());
                    addProperty(new_prop);
                } else {
                    printf("Could not find prototype estimator for attribute '%s'\n", AttributeConv::attribute_str(it->first).c_str());
                }
            }
        }
    }
}

// std::shared_ptr<SemanticObject> SemanticObject::clone() const {
//         //std::cout << "Semantic object: going to clone. This object = " << toString() << std::endl;
//     return std::make_shared<SemanticObject>(*this);
// }

const ClassModel& SemanticObject::getExpectedObjectModel() const {
    std::shared_ptr<const Property> class_prop = getProperty("class_label");

    string class_name;

    if (class_prop) {
        class_prop->getValue()->getExpectedValue(class_name);
    } else {
        class_name = "object";
    }

    return *KnowledgeDatabase::getInstance().getClassModel(class_name);
}

void SemanticObject::addPotentialAssignment(const Evidence& ev, double probability) {
    Assignment* assignment = new Assignment(Assignment::EXISTING, &ev, shared_from_this(), probability);

    for(set<Hypothesis*>::iterator it_hyp = parent_hypotheses_->begin(); it_hyp != parent_hypotheses_->end(); ++it_hyp) {
        Hypothesis& hyp = **it_hyp;
        hyp.addPotentialAssignment(assignment);
    }

    // TODO: check if object now has potential assignments between hypotheses of different clusters. If so, those trees need to merge.
}

ObjectID SemanticObject::getID() const {
    return ID_;
}

void SemanticObject::addToHypothesis(Hypothesis* hyp) {
    parent_hypotheses_->insert(hyp);
}

void SemanticObject::removeFromHypothesis(Hypothesis* hyp) {
//         std::cout << "parent_hypotheses_ = " << parent_hypotheses_ << " semantic object = " << this << std::endl;
//         std::cout << "Remove from Hypothesis: parent_hypotheses_.size() = " << parent_hypotheses_->size() << std::endl;
//         std::cout << "remove from hypothesis: hyp = " << hyp << std::endl;
//         std::cout << "parent hypothesis = " << std::endl;
//         for(std::set<Hypothesis*>::iterator it = parent_hypotheses_->begin(); it != parent_hypotheses_->end(); it++)
//         {
//                 Hypothesis* hypTest = *it;
//                 std::cout << hypTest << "\t";
//         }
//         std::cout << "\n";
        
//         std::cout << "removeFromHypothesis: # parent_hypotheses_ = "  << parent_hypotheses_->size() << std::endl;
    parent_hypotheses_->erase(hyp);
//     std::cout << "removeFromHypothesis, after removal: # parent_hypotheses_ = "  << parent_hypotheses_->size() << std::endl;
}

unsigned int SemanticObject::getNumParentHypotheses() const {
    return parent_hypotheses_->size();
}

std::set<Hypothesis*>* SemanticObject::getParentHypotheses(){
        return parent_hypotheses_;
}

std::string SemanticObject::toString() const
{
    std::stringstream s;
    
    s << "Semantic object: id = " << ID_ ;
    s << " expected class = " << expected_class_ ;
    s << " num parent Hyp = " << getNumParentHypotheses() << std::endl;//<< " parent_hypotheses_ = " << std::endl;
      
  //  for(std::set<Hypothesis*>::const_iterator it = parent_hypotheses_.begin(); it != parent_hypotheses_.end(); ++it) {
   //     s << " - " << *it << endl;
   // }
    return s.str();
}

}
