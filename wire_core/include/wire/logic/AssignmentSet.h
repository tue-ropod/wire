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
#include <wire/storage/SemanticObject.h> // TODO TEMP

namespace mhf {

class Assignment;
class Hypothesis;
class AssignmentMatrix;
// class SemanticObject; // TODO TEMP

class AssignmentSet {

public:

    AssignmentSet(Hypothesis* hyp, AssignmentMatrix* assignment_matrix);

    AssignmentSet(const AssignmentSet& orig);

    virtual ~AssignmentSet();

    void init();

    void expand(std::list<AssignmentSet*>& children) const;

    bool allMeasurementsAssigned() const;

    bool allObjectsAssigned() const;

    AssignmentSet* constructNextBest() const;

    const Assignment& getMeasurementAssignment(unsigned int i_ev) const;

    void getAllAssignments(std::list<const Assignment*>& assignments) const;

    double getProbability() const;

    Hypothesis* getHypothesis() const;

    int getNumMeasurements() const;

    bool isValid() const;
    
    bool multipleAssignmentsOfSingleEvidence() const;

    void print() const;

protected:

    Hypothesis* hyp_;

    AssignmentMatrix* assignment_matrix_;

    double probability_;

    std::vector<unsigned int> evidence_assignments_; // after storting the evidence probabilities, this gives the numbers in the assignment matrix to which object the evidence is applied.

    int n_blocked_;

};

}

#endif /* ASSIGNMENTSET_H_ */
