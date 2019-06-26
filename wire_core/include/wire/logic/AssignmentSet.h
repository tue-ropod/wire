/*
 * AssignmentSet.h
 *
 *  Created on: Jul 29, 2011
 *      Author: sdries
 */

#ifndef ASSIGNMENTSET_H_
#define ASSIGNMENTSET_H_

#include <list>
#include <vector>
#include <map>
#include <iostream>
#include <memory>

namespace mhf {

class Assignment;
class Hypothesis;
class AssignmentMatrix;
class SemanticObject;

class AssignmentSet {

public:

    AssignmentSet(std::shared_ptr<Hypothesis> hyp, std::shared_ptr<AssignmentMatrix> assignment_matrix);

    AssignmentSet(const AssignmentSet& orig);

    virtual ~AssignmentSet();

    void init();

    void expand(std::list<std::shared_ptr<AssignmentSet>>& children) const;

    bool allMeasurementsAssigned() const;

    bool allObjectsAssigned() const;

    std::shared_ptr<AssignmentSet> constructNextBest() const;

    std::shared_ptr<const Assignment> getMeasurementAssignment(unsigned int i_ev) const;

    void getAllAssignments(std::list<std::shared_ptr<const Assignment>>& assignments) const;

    double getProbability() const;

    std::shared_ptr<Hypothesis> getHypothesis() const;

    int getNumMeasurements() const;

    bool isValid() const;

    void print() const;

protected:

    std::shared_ptr<Hypothesis> hyp_;

    std::shared_ptr<AssignmentMatrix> assignment_matrix_;

    double probability_;

    std::vector<unsigned int> evidence_assignments_;

    int n_blocked_;

};

}

#endif /* ASSIGNMENTSET_H_ */
