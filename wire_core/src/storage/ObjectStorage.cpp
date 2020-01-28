#include "wire/storage/ObjectStorage.h"

#include "wire/storage/KnowledgeDatabase.h"
#include "wire/storage/SemanticObject.h"
#include "wire/core/Evidence.h"
#include <limits> // TEMP
#include "ros/ros.h"// TEMP

typedef std::numeric_limits< double > dbl;

using namespace std;

namespace mhf {

ObjectStorage* ObjectStorage::instance_ = 0;

ObjectStorage& ObjectStorage::getInstance() {
    if (instance_) {
        return *instance_;
    }
    instance_ = new ObjectStorage();
    return *instance_;
}

ObjectStorage::ObjectStorage() : ID_(0), knowledge_db_(KnowledgeDatabase::getInstance()) {
        
         objects_ = std::make_shared<std::list<std::shared_ptr<SemanticObject>>>();
}

ObjectStorage::~ObjectStorage() {

}

void ObjectStorage::addObject(std::shared_ptr<SemanticObject> obj) {
    objects_->push_back(obj);
    obj->it_obj_storage_ = --objects_->end();
}

std::list<std::shared_ptr<SemanticObject>>::iterator ObjectStorage::removeObject(SemanticObject& obj) {        
    std::list<std::shared_ptr<SemanticObject>>::iterator next_it = objects_->erase(obj.it_obj_storage_);
    return next_it;
}

long ObjectStorage::getUniqueID() {
    return ID_++;
}

void ObjectStorage::match(const Evidence& ev) {

//     std::list<SemanticObject> objects2Remove;
        
    for(list<std::shared_ptr<SemanticObject>>::iterator it_obj = objects_->begin(); it_obj != objects_->end(); ++it_obj) {
        SemanticObject& obj = **it_obj;
        
           obj.propagate(ev.getTimestamp()); // propagated to current timestamp, which is being set in process evidence of WorldModelROS.cpp
    }

    for(list<std::shared_ptr<SemanticObject>>::iterator it_obj = objects_->begin(); it_obj != objects_->end(); ++it_obj) {
        SemanticObject& obj = **it_obj;

        double prob_existing = KnowledgeDatabase::getInstance().getProbabilityExisting(ev, obj);
        if (prob_existing > 0) {
            obj.addPotentialAssignment(ev, prob_existing);
        }
    }
}

std::shared_ptr<std::list<std::shared_ptr<SemanticObject>>> ObjectStorage::getObjects() const
{
        return objects_;        
}

}
