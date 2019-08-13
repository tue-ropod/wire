/*
 * ObjectTest.h
 *
 *  Created on: Apr 27, 2011
 *      Author: sdries
 */

#ifndef MHT_SEMANTICOBJECT_H_
#define MHT_SEMANTICOBJECT_H_

#include "wire/core/PropertySet.h"
#include <list>

namespace mhf {

class ClassModel;
class Evidence;
class Assignment;
class Hypothesis;

class SemanticObject : public PropertySet, public std::enable_shared_from_this<SemanticObject> {

public:

   // static int N_SEMANTICOBJECT;

    std::list<std::shared_ptr<SemanticObject>>::iterator it_obj_storage_;

    SemanticObject(long ID);

    SemanticObject(const SemanticObject& orig);

    virtual ~SemanticObject();

    void init(std::shared_ptr<const Evidence> z);

    void update(std::shared_ptr<const Evidence> z);

    //std::shared_ptr<SemanticObject> clone() const;
    
    std::shared_ptr<IStateEstimator> clone() const{ return CloneMethod(); };
   
    std::shared_ptr<SemanticObject> CloneMethod() const {return std::make_shared< SemanticObject>(*this);}

    std::shared_ptr<SemanticObject> cloneThis() const {return std::make_shared< SemanticObject>(*this);}
    
    double getLastUpdateTime() const;

    double getTimestamp() const;

    std::shared_ptr<const ClassModel> getExpectedObjectModel() const;

    double getLikelihood( const std::shared_ptr< PropertySet> ev) const;

    void addPotentialAssignment(const std::shared_ptr< Evidence> ev, double probability);

    ObjectID getID() const;

    void addToHypothesis(std::shared_ptr<Hypothesis> hyp);

    void removeFromHypothesis(std::shared_ptr<Hypothesis> hyp);

    unsigned int getNumParentHypotheses() const;

protected:

    ObjectID ID_;

    std::string expected_class_;

    std::set<std::shared_ptr<Hypothesis>> parent_hypotheses_;

};

}

#endif /* SEMANTICOBJECT_H_ */
