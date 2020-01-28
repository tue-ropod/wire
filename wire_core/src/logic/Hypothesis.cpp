/*
 * Hypothesis.cpp
 *
 *  Created on: March, 2011
 *  Author: Jos Elfring, Sjoerd van den Dries
 *  Affiliation: Eindhoven University of Technology
 */

#include "wire/logic/Hypothesis.h"

#include "wire/logic/AssignmentSet.h"
#include "wire/logic/Assignment.h"
#include "wire/logic/AssignmentMatrix.h"
#include "wire/storage/SemanticObject.h"
#include "wire/storage/ObjectStorage.h"

using namespace std;

namespace mhf {

/* ****************************************************************************** */
/* *                        CONSTRUCTOR / DESTRUCTOR                            * */
/* ****************************************************************************** */

Hypothesis::Hypothesis(const double& timestamp, double probability) : probability_(probability), timestamp_(timestamp),
    parent_(0), assignment_set_(0), assignment_matrix_(0), height_(0), is_active_leaf_(true) {
            
//             std::cout << "Hypothesis constructor called: this hypothesis = " << this << std::endl;
            
          //  std::list<SemanticObject*>* initObjects;
            objects_ = new std::list<std::shared_ptr<SemanticObject>>();
//             std::cout << "hypothesis constructor: p objects = " << objects_ << std::endl;
}

Hypothesis::~Hypothesis() {
        
//         std::cout << "Hypothesis destructor called: this hypothesis = " << this << std::endl;
        
    clear();
    delete assignment_matrix_;
    delete objects_;
}

/* ****************************************************************************** */
/* *                                 GETTERS                                    * */
/* ****************************************************************************** */


const AssignmentSet* Hypothesis::getAssignments() const {
    return assignment_set_;
}

const Hypothesis* Hypothesis::getBestLeaf() const {
    return best_leaf_;
}


list<Hypothesis*>& Hypothesis::getChildHypotheses() {
    return children_;
}

int Hypothesis::getHeight() const {
    return height_;
}

int Hypothesis::getNumObjects() const {
    return objects_->size();
}

const list<std::shared_ptr<SemanticObject>>* Hypothesis::getObjects() const {
        
     //   std::cout << "Hypothesis = " << this << " objects_ = " ;//<< std::endl;
//         std::cout << objects_ << std::endl;
        
     //   std::cout << "Hypothesis inside Objects functionality, size = " ;
//         std::cout << objects_->size();
    //    std::cout << "Hypothesis numObjects = " << getNumObjects() ;//<< " numeric limit = " << std::numeric_limits<int>::max() << std::endl;
//          bool check = objects_->begin() == objects_->end();
//          int dist = std::distance(objects_->begin(), objects_->end() );
     //   std::cout << "Hypothesis::getObjects() : tests, size " <<  objects_->size() << " dist = " << dist  << " check = " << check << " prob = " << this->getProbability() << std::endl;       
        
    return objects_;
}


const Hypothesis* Hypothesis::getParent() const {
    return parent_;
}

double Hypothesis::getProbability() const {
    return probability_;
}


double Hypothesis::getTimestamp() const {
    return timestamp_;
}

AssignmentMatrix* Hypothesis::getAssignmentMatrix() const {
    return assignment_matrix_;
}

/* ****************************************************************************** */
/* *                                 SETTERS                                    * */
/* ****************************************************************************** */


void Hypothesis::setAssignments(AssignmentSet* assignments) {
    if (assignment_set_) {
        delete assignment_set_;
    }
    assignment_set_ = assignments;
}


void Hypothesis::setInactive() {
    is_active_leaf_ = false;
}

void Hypothesis::setProbability(double prob) {
    probability_ = prob;
}

/* ****************************************************************************** */
/* *                           HYPOTHESIS MODIFIERS                             * */
/* ****************************************************************************** */


void Hypothesis::addChildHypothesis(Hypothesis* h) {
    h->parent_ = this;
    this->is_active_leaf_ = false;
    children_.push_back(h);
}

void Hypothesis::addObject(std::shared_ptr<SemanticObject> obj) {
//     int numObjectsPrev = getNumObjects();
    objects_->push_back(obj);
//     std::cout << "going to add object to hypothesis, obj = " << obj->toString() << " current #objects = " << numObjectsPrev << " hypothesis = " << this << std::endl;
    obj->addToHypothesis(this);
//     int numObjectsAfter = getNumObjects();
//     std::cout << "object added to hypothesis, #objects = " << numObjectsAfter << std::endl;
    
//     if(numObjectsPrev - numObjectsAfter > 1) // TODO TEMP
//     {
//             std::cout << "Hypothesis::addObject PROBLEMSSSSS!!!" << std::endl;
//                 exit;
//     }
}

void Hypothesis::removeObject(std::shared_ptr<SemanticObject> obj) {
    objects_->remove(obj);
    obj->removeFromHypothesis(this);
}

void Hypothesis::clearAssignmentSet() {
    delete assignment_set_;
    assignment_set_ = 0;
}

void Hypothesis::addPotentialAssignment(Assignment* assignment) {
    if (!assignment_matrix_) {
        assignment_matrix_ = new AssignmentMatrix();
    }
    assignment_matrix_->addPotentialAssignment(*assignment);
}

void Hypothesis::applyAssignments() {

    list<const Assignment*> all_assignments;
    assignment_set_->getAllAssignments(all_assignments);

    // apply cases without target
    for(list<const Assignment*>::iterator it_ass = all_assignments.begin(); it_ass != all_assignments.end();) {
        const Assignment* ass = *it_ass;

        if (ass->getType() == Assignment::CLUTTER) {
            // remove assignment from list
            it_ass = all_assignments.erase(it_ass);
        } else if (ass->getType() == Assignment::NEW) {
            std::shared_ptr<SemanticObject> new_obj = ass->getNewObject();
            addObject(new_obj);

            // remove assignment from list
            it_ass = all_assignments.erase(it_ass);
        } else {
            ++it_ass;
        }
    }

    // apply cases with target
    const list<std::shared_ptr<SemanticObject>>* hyp_parent_objs = parent_->getObjects();
    for (list<std::shared_ptr<SemanticObject>>::const_iterator it_obj = hyp_parent_objs->begin(); it_obj != hyp_parent_objs->end(); ++it_obj) {
             
        std::shared_ptr<SemanticObject> obj = *it_obj;
        
        const Assignment* update_ass = 0;

        for(list<const Assignment*>::iterator it_ass = all_assignments.begin(); it_ass != all_assignments.end();) {
            const Assignment* ass = *it_ass;

            if (obj == ass->getTarget()) {
                update_ass = ass;
                it_ass = all_assignments.erase(it_ass);
            } else {
                ++it_ass;
            }
        }

        if (update_ass) {
            std::shared_ptr<SemanticObject> updated_obj = update_ass->getUpdatedObject();
            addObject(updated_obj);
        } else {
            addObject(obj);
        }
    }

    assert(all_assignments.empty());
    clearAssignmentSet();
}

/* ****************************************************************************** */
/* *                           TREE UPDATE METHODS                              * */
/* ****************************************************************************** */

double Hypothesis::calculateBranchProbabilities() {
    if (is_active_leaf_) {
        return probability_;
    }

    probability_ = 0;

    for (list<Hypothesis*>::const_iterator it = children_.begin(); it != children_.end(); ++it) {
        probability_ += (*it)->calculateBranchProbabilities();
    }
    return probability_;
}


int Hypothesis::calculateHeigth() {
    height_ = 0;
    for (list<Hypothesis*>::const_iterator it = children_.begin(); it != children_.end(); ++it) {
        height_ = max(height_, (*it)->calculateHeigth() + 1);
    }
    return height_;
}


Hypothesis* Hypothesis::determineBestLeaf() {
    if (is_active_leaf_) {
        return this;
    }
    best_leaf_ = 0;
    for (list<Hypothesis*>::const_iterator it = children_.begin(); it != children_.end(); ++it) {
        Hypothesis* child_best_leaf_ = (*it)->determineBestLeaf();
        if (best_leaf_ == 0 || (child_best_leaf_ != 0 && child_best_leaf_->getProbability() > best_leaf_->getProbability())) {
            best_leaf_ = child_best_leaf_;
        }
    }
    return best_leaf_;

}


void Hypothesis::findActiveLeafs(list<Hypothesis*>& active_leafs) {
    if (is_active_leaf_) {
        active_leafs.push_back(this);
        return;
    }

    for (list<Hypothesis*>::const_iterator it = children_.begin(); it != children_.end(); ++it) {
        (*it)->findActiveLeafs(active_leafs);
    }

}

/* ****************************************************************************** */
/* *                       TREE CLEAR / DELETE METHODS                          * */
/* ****************************************************************************** */


void Hypothesis::clear() {
    // remove this hypothesis from the hypothesis list of all objects contained in this hypothesis
       // std::cout << "Remove hypothesis: " << std::endl;
        
//         std::cout << "Hypothesis clear: objects_ = " << objects_ << std::endl;
//         std::cout << "Hypothsis clear: objects_size = " << objects_->size() << std::endl;
//         bool check = objects_->begin() ==  objects_->end();
//         std::cout << "Hypothesis clear: check = " << check << std::endl;
        
    for (list<std::shared_ptr<SemanticObject>>::iterator it_obj = objects_->begin(); it_obj != objects_->end(); ++it_obj) {
        std::shared_ptr<SemanticObject> obj = *it_obj;
        obj->removeFromHypothesis(this);
        
//         std::cout << "Hypothesis clear(): obj->getNumParentHypotheses() = " << obj->getNumParentHypotheses() << std::endl;
        
        if (obj->getNumParentHypotheses() == 0) {
            ObjectStorage::getInstance().removeObject(*obj);
         //   std::cout << "Going to delete object " << obj->toString() << std::endl;
         //   delete obj;
        }
    }

    //std::cout << "Going to clear objects in hypothesis, #objects = " << getNumObjects() << std::endl;
//     std::cout << "Going to clear objects in hypothesis " << std::endl;
    objects_->clear();
    //std::cout << "objects cleared, #objects = " << getNumObjects() << std::endl;

    is_active_leaf_ = false;

    // remove any remaining assignments
    if (assignment_set_) {
        delete assignment_set_;
    }
    
   /* if(objects_){
            std::cout << "objects_ " << objects_ << " of hypothesis " << this << " is going to be deleted" << std::endl;
            
        delete objects_;
        objects_ = 0;
    }
    */
}


void Hypothesis::clearInactive() {
//         std::cout << "clear inactive" << std::endl;
        
    if (!is_active_leaf_) {
        clear();
        for (list<Hypothesis*>::const_iterator it = children_.begin(); it != children_.end(); ++it) {
            (*it)->clearInactive();
        }
    }
}


void Hypothesis::deleteChildren() {
    // delete all child hypotheses
    for (list<Hypothesis*>::iterator it = children_.begin(); it != children_.end(); ++it) {
        (*it)->deleteChildren();
        
//         std::cout << "delete children: hyp " << *it << " is going to be deleted" << std::endl;
        
        delete (*it);
    }
}


Hypothesis* Hypothesis::deleteSinglePaths() {
    for (list<Hypothesis*>::iterator it_child = children_.begin(); it_child != children_.end();) {
        Hypothesis* new_child = (*it_child)->deleteSinglePaths();
        if (new_child != *it_child) {
            it_child = children_.erase(it_child);
            children_.insert(it_child, new_child);
        } else {
            ++it_child;
        }
    }

    if (children_.size() == 1) {
        Hypothesis* hyp = *children_.begin();
        hyp->parent_ = this->parent_;
        
//         std::cout << "delete deleteSinglePaths: hyp " << this << " is going to be deleted" << std::endl;
        
        delete this;
        return hyp;
    }

    return this;
}

}
