/*
 * Assignment.h
 *
 *  Created on: Jul 28, 2011
 *      Author: sdries
 */

#include "wire/logic/Assignment.h"

#include "wire/storage/KnowledgeDatabase.h"
#include "wire/storage/SemanticObject.h"
#include "wire/storage/ObjectStorage.h"

namespace mhf {

Assignment::Assignment(AssignmentType type , const Evidence* evidence, std::shared_ptr<const SemanticObject> target, double probability)
    : type_(type), evidence_(evidence), target_(target), probability_(probability), new_object_(0), updated_object_(0) {

}

Assignment::~Assignment() {
}

Assignment::AssignmentType Assignment::getType() const {
    return type_;
}

const Evidence* Assignment::getEvidence() const {
    return evidence_;
}

std::shared_ptr<const SemanticObject> Assignment::getTarget() const {
    return target_;
}

double Assignment::getProbability() const {
    return probability_;
}


std::shared_ptr<SemanticObject> Assignment::getNewObject() const {
    if (new_object_) {
        return new_object_;
    }
    new_object_ = std::make_shared<SemanticObject>(ObjectStorage::getInstance().getUniqueID());
    new_object_->init(*evidence_);

    ObjectStorage::getInstance().addObject(new_object_);
    return new_object_;
}

std::shared_ptr<SemanticObject> Assignment::getUpdatedObject() const {
    if (updated_object_) {
        return updated_object_;
    }
    updated_object_ = target_->cloneThis();
    updated_object_->update(*evidence_);

    ObjectStorage::getInstance().addObject(updated_object_);
    return updated_object_;
}

std::string Assignment::toString() const {
    std::stringstream ss;
    ss << "Evidence " << evidence_ << " -> ";
    if (type_ == Assignment::CLUTTER) {
        ss << "CLUTTER";
    } else if (type_ == Assignment::NEW) {
        ss << "NEW";
    } else if (type_ == Assignment::EXISTING) {
        ss << "Object " << target_;
        ss << " with ID = " << target_->getID();
    }
     ss << " probability = " << probability_;
    return ss.str();
}

}
