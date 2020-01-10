/*
 * Predicate.cpp
 *
 *  Created on: Jan 12, 2012
 *      Author: sdries
 */

#include "wire/core/Property.h"
#include "wire/core/IStateEstimator.h"

namespace mhf {

Property::Property(const Attribute& attribute, const IStateEstimator& bm, const ObjectID& object_id)
    : time_(0), attribute_(attribute), estimator_(bm.clone()) {
}

Property::Property(const Property& orig)
    : time_(orig.time_), attribute_(orig.attribute_), estimator_(orig.estimator_->clone()) {
}

Property::~Property() {
    delete estimator_;
}

Property* Property::clone() const {
    return new Property(*this);
}

Property& Property::operator=(const Property& other) {
    if (this != &other) {
        delete estimator_;
        time_ = other.time_;
        attribute_ = other.attribute_;
        estimator_ = other.estimator_->clone();
    }

    return *this;
}

const Attribute& Property::getAttribute() const {
    return attribute_;
}

const IStateEstimator& Property::getEstimator() const {
    return *estimator_;
}

std::shared_ptr<const pbl::PDF> Property::getValue() const {
    return estimator_->getValue();
}

std::shared_ptr<const pbl::PDF> Property::getFullValue() const {
    return estimator_->getFullValue();
}

void Property::update(std::shared_ptr<const pbl::PDF> z, const Time& time) {
     if (time < time_)
    {
                std::cout << "Timing issues!!" << std::endl;
            return;
    }
    estimator_->update(z, time);
    time_ = time;
    latestUpdateTime_ = time;
}

void Property::propagate(const Time& time) {

    if (time < time_) return;
    
    std::cout << "Property::propagate. going to propagate estimator " << std::endl;
    estimator_->propagate(time);
    time_ = time;
}

void Property::reset() {
    estimator_->reset();
}

double Property::getLikelihood(std::shared_ptr<const pbl::PDF> pdf) const {
    return estimator_->getValue()->getLikelihood(pdf);
}

std::string Property::toString(const std::string& prefix) const {
        if(estimator_->getValue())
        {
                return estimator_->getValue()->toString();
        } 
        else
        {
                return "Property equals nullptr \n";
        }
}
    

}
