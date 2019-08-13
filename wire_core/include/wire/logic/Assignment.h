/*
 * Assignment.h
 *
 *  Created on: Jul 28, 2011
 *      Author: sdries
 */

#ifndef WM_ASSIGNMENT_H_
#define WM_ASSIGNMENT_H_

#include <string>
#include <iostream> 
#include <memory>

namespace mhf {

class Evidence;
class SemanticObject;

class Assignment: public std::enable_shared_from_this<Assignment>{

public:

    enum AssignmentType {
        NEW,
        EXISTING,
        CLUTTER
    };

    virtual ~Assignment();

    Assignment(AssignmentType type , const std::shared_ptr< Evidence> evidence, const std::shared_ptr< SemanticObject> target, double probability);

    AssignmentType getType() const;

    std::shared_ptr<const Evidence> getEvidence() const;

    std::shared_ptr<const SemanticObject> getTarget() const;

    double getProbability() const;

    std::shared_ptr<SemanticObject> getNewObject() const;

    std::shared_ptr<SemanticObject> getUpdatedObject() const;

    std::string toString() const;

protected:

    AssignmentType type_;

    std::shared_ptr<const Evidence> evidence_;

    std::shared_ptr<const SemanticObject> target_;

    double probability_;

    //mutable SemanticObject* new_object_;
    //mutable std::shared_ptr<SemanticObject> new_object_;
    mutable std::shared_ptr<SemanticObject> new_object_;

    // mutable SemanticObject* updated_object_;
    mutable std::shared_ptr<SemanticObject> updated_object_;
};

}

#endif /* ASSIGNMENT_H_ */
