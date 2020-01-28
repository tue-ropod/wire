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

class SemanticObject : public PropertySet, public std::enable_shared_from_this<SemanticObject>  {

public:

    static int N_SEMANTICOBJECT;

    std::list<std::shared_ptr<SemanticObject>>::iterator it_obj_storage_;

    SemanticObject(long ID);

    SemanticObject(const SemanticObject& orig);

    virtual ~SemanticObject();

    void init(const Evidence& z);

    void update(const Evidence& z);

//     std::shared_ptr<SemanticObject> clone() const;
    
    virtual std::shared_ptr<IStateEstimator> clone() const { return cloneThis(); };
  
    std::shared_ptr<SemanticObject> cloneThis() const 
    {             
            return std::make_shared< SemanticObject>(*this);
    }
    
    //double getLastUpdateTime() const { return };

    //double getTimestamp() const;

    const ClassModel& getExpectedObjectModel() const;

    double getLikelihood(const PropertySet& ev) const;

    void addPotentialAssignment(const Evidence& ev, double probability);

    ObjectID getID() const;

    void addToHypothesis(Hypothesis* hyp);

    void removeFromHypothesis(Hypothesis* hyp);

    unsigned int getNumParentHypotheses() const;
    
    std::set<Hypothesis*>* getParentHypotheses();
    
    std::string toString() const;

protected:

    ObjectID ID_;

    std::string expected_class_;

    std::set<Hypothesis*>* parent_hypotheses_;

};

}

#endif /* SEMANTICOBJECT_H_ */
