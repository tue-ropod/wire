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
}

Hypothesis::~Hypothesis() {
    clear();
   // delete assignment_matrix_;
}

/* ****************************************************************************** */
/* *                                 GETTERS                                    * */
/* ****************************************************************************** */


std::shared_ptr<const AssignmentSet> Hypothesis::getAssignments() const {
    return assignment_set_;
}

std::shared_ptr<const Hypothesis> Hypothesis::getBestLeaf() const {
    return best_leaf_;
}


list<std::shared_ptr<Hypothesis>>& Hypothesis::getChildHypotheses() {
    return children_;
}

int Hypothesis::getHeight() const {
    return height_;
}

int Hypothesis::getNumObjects() const {
    return objects_.size();
}

const list<std::shared_ptr<SemanticObject>>& Hypothesis::getObjects() const {
    return objects_;
}


std::shared_ptr<const Hypothesis> Hypothesis::getParent() const {
    return parent_;
}

double Hypothesis::getProbability() const {
    return probability_;
}


double Hypothesis::getTimestamp() const {
    return timestamp_;
}

std::shared_ptr<AssignmentMatrix> Hypothesis::getAssignmentMatrix() const {
    return assignment_matrix_;
}

/* ****************************************************************************** */
/* *                                 SETTERS                                    * */
/* ****************************************************************************** */


void Hypothesis::setAssignments(std::shared_ptr<AssignmentSet> assignments) {
//     if (assignment_set_) {
//        delete assignment_set_;
//     }
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


void Hypothesis::addChildHypothesis(std::shared_ptr<Hypothesis> h) {
    h->parent_ = shared_from_this();
    this->is_active_leaf_ = false;
    children_.push_back(h);
}

void Hypothesis::addObject(std::shared_ptr<SemanticObject> obj) {
    objects_.push_back(obj);
    obj->addToHypothesis(shared_from_this());
}


void Hypothesis::clearAssignmentSet() {
  //  delete assignment_set_;
    assignment_set_ = 0;
}

void Hypothesis::addPotentialAssignment(std::shared_ptr<Assignment> assignment) {
    if (!assignment_matrix_) {
        assignment_matrix_ = std::make_shared<AssignmentMatrix>();
    }
    assignment_matrix_->addPotentialAssignment(assignment);
}

void Hypothesis::applyAssignments() {

    list<std::shared_ptr<const Assignment>> all_assignments;
    assignment_set_->getAllAssignments(all_assignments);

    // apply cases without target
    for(list<std::shared_ptr<const Assignment>>::iterator it_ass = all_assignments.begin(); it_ass != all_assignments.end();) {
        std::shared_ptr<const Assignment> ass = *it_ass;

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
    const list<std::shared_ptr<SemanticObject>>& hyp_parent_objs = parent_->getObjects();
    for (list<std::shared_ptr<SemanticObject>>::const_iterator it_obj = hyp_parent_objs.begin(); it_obj != hyp_parent_objs.end(); ++it_obj) {
        std::shared_ptr<SemanticObject> obj = *it_obj;

        std::shared_ptr<const Assignment> update_ass = 0;

        for(list<std::shared_ptr<const Assignment>>::iterator it_ass = all_assignments.begin(); it_ass != all_assignments.end();) {
            std::shared_ptr<const Assignment> ass = *it_ass;

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

    for (list<std::shared_ptr<Hypothesis>>::const_iterator it = children_.begin(); it != children_.end(); ++it) {
        probability_ += (*it)->calculateBranchProbabilities();
    }
    return probability_;
}


int Hypothesis::calculateHeigth() {
    height_ = 0;
    for (list<std::shared_ptr<Hypothesis>>::const_iterator it = children_.begin(); it != children_.end(); ++it) {
        height_ = max(height_, (*it)->calculateHeigth() + 1);
    }
    return height_;
}


std::shared_ptr<Hypothesis> Hypothesis::determineBestLeaf() {
    if (is_active_leaf_) {
        return shared_from_this();
    }
    best_leaf_ = 0;
    for (list<std::shared_ptr<Hypothesis>>::const_iterator it = children_.begin(); it != children_.end(); ++it) {
        std::shared_ptr<Hypothesis> child_best_leaf_ = (*it)->determineBestLeaf();
        if (best_leaf_ == 0 || (child_best_leaf_ != 0 && child_best_leaf_->getProbability() > best_leaf_->getProbability())) {
            best_leaf_ = child_best_leaf_;
        }
    }
    return best_leaf_;

}


void Hypothesis::findActiveLeafs(list<std::shared_ptr<Hypothesis>>& active_leafs) {
    if (is_active_leaf_) {
        active_leafs.push_back(shared_from_this());
        return;
    }

    for (list<std::shared_ptr<Hypothesis>>::const_iterator it = children_.begin(); it != children_.end(); ++it) {
        (*it)->findActiveLeafs(active_leafs);
    }

}

/* ****************************************************************************** */
/* *                       TREE CLEAR / DELETE METHODS                          * */
/* ****************************************************************************** */


void Hypothesis::clear() {
    // remove this hypothesis from the hypothesis list of all objects contained in this hypothesis
    for (list<std::shared_ptr<SemanticObject>>::iterator it_obj = objects_.begin(); it_obj != objects_.end(); ++it_obj) {
        std::shared_ptr<SemanticObject> obj = *it_obj;
        obj->removeFromHypothesis(shared_from_this());
        if (obj->getNumParentHypotheses() == 0) {
            ObjectStorage::getInstance()->removeObject(obj);
            
            obj = 0;
            //delete obj;
        }
    }

    objects_.clear();

    is_active_leaf_ = false;

    // remove any remaining assignments
    if (assignment_set_) {
            assignment_set_ = 0;
       // delete assignment_set_;
    }
}


void Hypothesis::clearInactive() {
    if (!is_active_leaf_) {
        clear();
        for (list<std::shared_ptr<Hypothesis>>::const_iterator it = children_.begin(); it != children_.end(); ++it) {
            (*it)->clearInactive();
        }
    }
}


void Hypothesis::deleteChildren() {
    // delete all child hypotheses
    for (list<std::shared_ptr<Hypothesis>>::iterator it = children_.begin(); it != children_.end(); ++it) {
        (*it)->deleteChildren();
   //     delete (*it);
    }
}


std::shared_ptr<Hypothesis> Hypothesis::deleteSinglePaths() {
    for (list<std::shared_ptr<Hypothesis>>::iterator it_child = children_.begin(); it_child != children_.end();) {
        std::shared_ptr<Hypothesis> new_child = (*it_child)->deleteSinglePaths();
        if (new_child != *it_child) {
            it_child = children_.erase(it_child);
            children_.insert(it_child, new_child);
        } else {
            ++it_child;
        }
    }

    if (children_.size() == 1) {
        std::shared_ptr<Hypothesis> hyp = *children_.begin();
        hyp->parent_ = this->parent_;
      //  delete this;
        return hyp;
    }

    return shared_from_this();
}

}
