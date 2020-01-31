/*
 * HypothesesTree.cpp
 *
 *  Created on: March, 2011
 *  Author: Jos Elfring, Sjoerd van den Dries
 *  Affiliation: Eindhoven University of Technology
 */

#include "wire/logic/HypothesesTree.h"

#include "wire/logic/Assignment.h"
#include "wire/logic/AssignmentSet.h"
#include "wire/logic/AssignmentMatrix.h"
#include "wire/logic/Hypothesis.h"

#include "wire/storage/KnowledgeDatabase.h"
#include "wire/storage/ObjectStorage.h"
#include "wire/storage/SemanticObject.h"

#include "wire/core/Evidence.h"
#include "wire/core/EvidenceSet.h"
#include "wire/core/Property.h"
#include "wire/core/ClassModel.h"

#include <ros/console.h>

#include <queue>
#include <cassert>
#include <float.h>

#ifdef MHF_MEASURE_TIME
    #include <time.h>
#endif

//#define DEBUG_INFO(_msg, ...) printf(_msg, ##__VA_ARGS__)
//#define DEBUG_INFO(_msg, ...)

namespace mhf {

/* ****************************************************************************** */
/* *                        CONSTRUCTOR / DESTRUCTOR                            * */
/* ****************************************************************************** */

HypothesisTree::HypothesisTree(int num_max_hyps, double max_min_prob_ratio) : n_updates_(0), t_last_update_(-1),
        tree_height_(0), num_max_hyps_(num_max_hyps), max_min_prob_ratio_(max_min_prob_ratio) {

    // create empty hypothesis (contains no objects) with timestep 0
    Hypothesis* empty_hyp = new Hypothesis(t_last_update_, 1.0);

    // add empty hypothesis to leaf list and set as root
    leafs_.push_back(empty_hyp);
    root_ = empty_hyp;
    MAP_hypothesis_ = empty_hyp;
    
//     std::cout << "MAP_hypothesis_ initialized at " << MAP_hypothesis_ << " and max_min_prob_ratio_ " << max_min_prob_ratio_ << std::endl;
}

HypothesisTree::~HypothesisTree() {
    root_->deleteChildren();
    delete root_;
}

/* ****************************************************************************** */
/* *                          PUBLIC MHT OPERATIONS                             * */
/* ****************************************************************************** */

void HypothesisTree::addEvidence(const EvidenceSet& ev_set) {
    ROS_DEBUG("HypothesesTree::processMeasurements\n");

    if (ev_set.size() == 0) {
        return;
    }

#ifdef MHF_MEASURE_TIME
    timespec t_start_total, t_end_total;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_start_total);
#endif

    //** Propagate all objects, compute association probabilities and add all possible measurement-track assignments
    // assumption: all evidence from the set originates from the same time-sample
    for(EvidenceSet::const_iterator it_ev = ev_set.begin(); it_ev != ev_set.end(); ++it_ev) {
        ObjectStorage::getInstance().match(**it_ev); // In here, the associations to existing objects are solved -> addPotentialAssignment in match-method.
    }

    t_last_update_ = ev_set.getTimestamp();

//     std::cout << "expand tree " << std::endl;
    expandTree(ev_set);
//     std::cout << "expand tree finished " << std::endl;

//     std::cout << "prune tree" << std::endl;
    pruneTree(ev_set.getTimestamp());
//     std::cout << "prunte tree finished" << std::endl;

    applyAssignments();

    // clear old hypotheses leafs
    // The hypotheses will still be there to form a tree, but do not contain any objects anymore
    root_->clearInactive();

    root_ = root_->deleteSinglePaths();

    tree_height_ = root_->calculateHeigth();
    
    ROS_DEBUG("*** Free memory: assignment matrices ***\n");

    ++n_updates_;

#ifdef MHF_MEASURE_TIME
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_end_total);
    printf("Total update took %f seconds.\n", (t_end_total.tv_sec - t_start_total.tv_sec) + double(t_end_total.tv_nsec - t_start_total.tv_nsec) / 1e9);
#endif

    ROS_DEBUG("HypothesesTree::processMeasurements - end\n");
}

void HypothesisTree::removeOldObjects(Time timeConstraint) {

    // check all objects which haven't been updated for a long period 
    std::shared_ptr<std::list<std::shared_ptr<SemanticObject>>> allObjects = ObjectStorage::getInstance().getObjects(); 
    
    for (std::list<std::shared_ptr<SemanticObject>>::iterator it_obj = allObjects->begin(); it_obj != allObjects->end(); ++it_obj) 
    {        
        std::shared_ptr<SemanticObject> obj = *it_obj;
        Time latestUpdate = obj->getLatestUpdateTime();
        bool objectRemoved = false;
            
        if(obj->getLatestUpdateTime() < timeConstraint)
        {
            // object too old: remove from all hypotheses
            for(std::list<Hypothesis*>::iterator it_hyp = leafs_.begin(); it_hyp !=  leafs_.end() && !objectRemoved; it_hyp++)
            {
                 const std::list<std::shared_ptr<SemanticObject>>* hypothesisObjects =  (*it_hyp)->getObjects();
                 Hypothesis* hyp = *it_hyp;
                 hyp->removeObject(obj);
                
                 if (obj->getNumParentHypotheses() == 0) 
                 {
                     std::list<std::shared_ptr<SemanticObject>>::iterator next_it = ObjectStorage::getInstance().removeObject(*obj);       
                     it_obj = next_it;
                     objectRemoved = true;
                 }
             }
                     
             if(objectRemoved)
             {
                  objectRemoved = false;
                  continue;
             }
         }
    }
}

/* ****************************************************************************** */
/* *                         PROTECTED MHT OPERATIONS                           * */
/* ****************************************************************************** */

struct compareAssignmentSets {
    bool operator()(const AssignmentSet* a1, const AssignmentSet* a2) const {
        return a1->getProbability() < a2->getProbability();
   }
};

void HypothesisTree::applyAssignments() {
    ROS_DEBUG("applyAssignments - begin\n");

    // iterate over all leaf hypotheses
    for (std::list<Hypothesis*>::iterator it = leafs_.begin(); it != leafs_.end(); ++it) {
        ROS_DEBUG("  materializing hyp %p, with parent %p\n", (*it), (*it)->getParent());
        (*it)->applyAssignments();
    }

    ROS_DEBUG("applyAssignments - end\n");
}

void HypothesisTree::expandTree(const EvidenceSet& ev_set) {

    ROS_DEBUG("expandTree - start\n");

    //** Create new objects based on measurements

    std::list<Assignment*> new_assignments;
    std::list<Assignment*> clutter_assignments;
    for(EvidenceSet::const_iterator it_ev = ev_set.begin(); it_ev != ev_set.end(); ++it_ev) {
        // new
        new_assignments.push_back(new Assignment(Assignment::NEW, *it_ev, 0, KnowledgeDatabase::getInstance().getProbabilityNew(**it_ev)));

        // clutter
        clutter_assignments.push_back(new Assignment(Assignment::CLUTTER, *it_ev, 0, KnowledgeDatabase::getInstance().getProbabilityClutter(**it_ev)));
    }

#ifdef MHF_MEASURE_TIME
    timespec t_start, t_end;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_start);
#endif

    //** expand all current leaf hypotheses

    std::priority_queue<AssignmentSet*, std::vector<AssignmentSet*>, compareAssignmentSets > assignment_sets;

    
    ROS_DEBUG(" - Create assignment matrices and assignment sets\n");

    for (std::list<Hypothesis*>::iterator it_hyp = leafs_.begin(); it_hyp != leafs_.end(); ++it_hyp) {
        Hypothesis* hyp = *it_hyp;

        // add new object assignments to current hypothesis
        for(std::list<Assignment*>::iterator it_ass = new_assignments.begin(); it_ass != new_assignments.end(); ++it_ass) {
            hyp->addPotentialAssignment(*it_ass);
        }

        // add clutter assignments to current hypothesis
        for(std::list<Assignment*>::iterator it_ass = clutter_assignments.begin(); it_ass != clutter_assignments.end(); ++it_ass) {
            hyp->addPotentialAssignment(*it_ass);
        }

        // evidence-to-object assignments are added in addEvidence() method

        // sort assignment matrix based on assignment probabilities
        hyp->getAssignmentMatrix()->sortAssignments();

        // create empty assignment set, add assignments and update probability (product hypothesis and assignment probs)
        AssignmentSet* ass_set = new AssignmentSet(hyp, hyp->getAssignmentMatrix());
            ROS_DEBUG(" ass_set.getNumMeasurements = %i\n", ass_set->getNumMeasurements());
        assignment_sets.push(ass_set);
    }

    ROS_DEBUG(" - Generate hypotheses\n");

    double min_prob = 0;

    // set all current leafs to inactive
    for (std::list<Hypothesis*>::iterator it_hyp = leafs_.begin(); it_hyp != leafs_.end(); ++it_hyp) {
        (*it_hyp)->setInactive();
    }

    leafs_.clear();

    int n_iterations = 0;
    
    std::cout << "EvidenceSet = " << ev_set.toString() << std::endl;

    // add hypotheses as long as there are criteria are met
    while(!assignment_sets.empty() && leafs_.size() < num_max_hyps_ && assignment_sets.top()->getProbability() > min_prob) {
        assignment_sets.top()->print();

        // Get most probable assignment
        ++n_iterations;
        ROS_DEBUG("  #assignment sets = %i\n", (int)assignment_sets.size());
        AssignmentSet* ass_set = assignment_sets.top();

        ROS_DEBUG("  inspecting assignment set %p with probability %.16f\n", ass_set, ass_set->getProbability());

        // remove most probable assignment (stored in ass_set pointer above)
        assignment_sets.pop();
        Hypothesis* hyp = ass_set->getHypothesis();
        ROS_DEBUG(" Before: ass_set.getNumMeasurements() = %i\n", ass_set->getNumMeasurements() );
        ROS_DEBUG(" Parent hypothesis: num objects = %i, probability = %f, timestamp = %f, \n", hyp->getNumObjects(), hyp->getProbability(), hyp->getTimestamp() );

            //std::cout << "hyp->getChildHypotheses().size() = " << hyp->getChildHypotheses().size() << std::endl;
          
        if (ass_set->isValid()) {
            /* ************ assignment set is complete! Create hypothesis ************ */
            
//             std::cout << "ass_set->getProbability() = " << ass_set->getProbability() << std::endl;
            
            Hypothesis* hyp_child = new Hypothesis(ev_set.getTimestamp(), ass_set->getProbability());
            ROS_DEBUG(" ass_set.getNumMeasurements() = %i\n", ass_set->getNumMeasurements() );
//             std::cout << " ass_set.getNumMeasurements() = " << ass_set->getNumMeasurements() << std::endl;
            
            hyp_child->setAssignments(ass_set);
            hyp->addChildHypothesis(hyp_child);

//             std::cout << "leafs_.empty() = " << leafs_.empty() << " leafs_.size() = " << leafs_.size() << std::endl;
            
            if (leafs_.empty()) {
                // first hypothesis found (and therefore the best one)
//                     std::cout << "max_min_prob_ratio_ = " << max_min_prob_ratio_ << std::endl;
//                     std::cout << "hyp_child->getProbability() = " << hyp_child->getProbability() << std::endl;
                    
                min_prob = hyp_child->getProbability() * max_min_prob_ratio_;

                MAP_hypothesis_ = hyp_child;
                
//                 std::cout << "MAP_hypothesis_ set to " << MAP_hypothesis_ << " min prob = " << min_prob << std::endl;
                
            }

            ROS_DEBUG(" NEW LEAF: %p\n", hyp_child);
//             std::cout << " NEW LEAF: " << hyp_child << std::endl;
            leafs_.push_back(hyp_child);
         //   ROS_DEBUG("  #leafs = %i, #old leafs = %i\n", (int)leafs_.size(), n_old_leafs);
            
            //ROS_DEBUG("  #leafs = %i\n", (int)leafs_.size());
            //std::printf("  #leafs = %i\n", (int)leafs_.size());

            /* ************************************************************************* */
        }

        std::list<AssignmentSet*> child_assignment_sets;
        ass_set->expand(child_assignment_sets);
        
        int counter = 0;
        for(std::list<AssignmentSet*>::iterator it_child = child_assignment_sets.begin(); it_child != child_assignment_sets.end(); ++it_child) {
            assignment_sets.push(*it_child);
        }
        
          ROS_DEBUG("child_assignment_sets.size = %i \n", counter);

    }

    ROS_DEBUG(" - Free memory (remaining assignment sets)\n");

    assert(leafs_.size() > 0);

    // delete remaining assignment sets (the ones that where not used to generate hypotheses)
    while(!assignment_sets.empty()) {
        delete assignment_sets.top();
        assignment_sets.pop();
    }

    ROS_DEBUG(" ... done\n");

    ++tree_height_;

    normalizeProbabilities();
    
    ROS_DEBUG("  #new_assignments = %i\n", (int)new_assignments.size());
    ROS_DEBUG("  #clutter_assignments = %i\n", (int)clutter_assignments.size());

#ifdef MHF_MEASURE_TIME
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_end);
    printf("Expansion of hypotheses took %f seconds.\n", (t_end.tv_sec - t_start.tv_sec) + double(t_end.tv_nsec - t_start.tv_nsec) / 1e9);
#endif

    ROS_DEBUG("expandTree - done\n");
}

void HypothesisTree::normalizeProbabilities() {
    // Calculate sum of all probabilities
    double p_total = 0;
    for(std::list<Hypothesis* >::iterator it_hyp = leafs_.begin(); it_hyp != leafs_.end(); ++it_hyp) {
        p_total += (*it_hyp)->getProbability();
    }

    // Normalize all probabilities
    for(std::list<Hypothesis* >::iterator it_hyp = leafs_.begin(); it_hyp != leafs_.end(); ++it_hyp) {
        (*it_hyp)->setProbability((*it_hyp)->getProbability() / p_total);
    }

    root_->calculateBranchProbabilities();
}

void HypothesisTree::pruneTree(const Time& timestamp) {
    //return;


    ROS_DEBUG("pruneTree - begin\n");

    ROS_DEBUG("   old #leafs = %i\n", (int)leafs_.size());

    // determine best branch leaf per hypothesis

    ROS_DEBUG(" - determine best leaf per branch\n");
    root_->determineBestLeaf();
    ROS_DEBUG("   ... done\n");

    double prob_ratios[] = {1e-8, 1e-7, 1e-6, 1e-5, 1e-5, 1e-4, 1};

    std::list<Hypothesis*> hyp_stack;
    hyp_stack.push_back(root_);

    while(!hyp_stack.empty()) {
        Hypothesis* hyp = hyp_stack.front();
        hyp_stack.pop_front();

        std::list<Hypothesis*>& children = hyp->getChildHypotheses();
        if (!children.empty()) {

            // determine best branch of root hypothesis (highest sum of leaf probabilities)
            Hypothesis* best_child = *children.begin();
            for (std::list<Hypothesis*>::const_iterator it_child = children.begin(); it_child != children.end(); ++it_child) {
                if ((*it_child)->getProbability() > best_child->getProbability()) {
                    best_child = *it_child;
                }
                //printf(" (%p, %f)", *it_child, (*it_child)->getProbability());
            }

            double prob_ratio = 0;

            if (hyp->getHeight() > 6) {
                prob_ratio = 0;
            } else {
                prob_ratio = prob_ratios[hyp->getHeight()];
            }

//             std::cout << "best_child->getProbability() = " << best_child->getProbability() << " best child = " << best_child << " parent of best child = " << best_child->getParent() << std::endl;
//             std::cout << " prob_ratio = " << prob_ratio << std::endl;
            
            double min_prob = best_child->getProbability() * prob_ratio;

            for (std::list<Hypothesis*>::iterator it_child = children.begin(); it_child != children.end();) {
                bool prune_child = false;
// std::cout << "it_child = " << *it_child << std::endl;
                if (*it_child != best_child) {
                    if ((*it_child)->getProbability() == 0) {
                      //      std::cout << "(*it_child)->getProbability() == 0" << std::endl;
                        prune_child = true;
                    } else if (hyp->getHeight() > 6) {
                        ROS_DEBUG(" - Determine hyp similarity between %p and %p\n", best_child->getBestLeaf(), (*it_child)->getBestLeaf());
                        double similarity = 1;
                        ROS_DEBUG("   ... done\n");

                        //printf("  similarity = %f\n", similarity);
                        bool test = (similarity > 0.5);
//     std::cout << "prune_child = (similarity > 0.5) = " << test << std::endl;
                        prune_child = (similarity > 0.5);
                    } else if ((*it_child)->getProbability() < min_prob) {
//                             std::cout << "(*it_child)->getProbability() < min_prob), (*it_child)->getProbability() = " << (*it_child)->getProbability() << " min prob = " << min_prob << std::endl;
                        prune_child = true;
                    }
                }

                
                if (prune_child) {
//                         std::cout << "hypothesisTree: prune child" << std::endl;
                    // prune hypothesis and all child hypothesis
                    (*it_child)->deleteChildren();
//                     std::cout << "Hyp " << *it_child << " is going to be deleted." << std::endl;
                    
                    delete (*it_child);
                    it_child = children.erase(it_child);
                } else {
                    hyp_stack.push_front(*it_child);
                    ++it_child;
                }
            }

        }
    }

    // clear leaf list and add new leafs of tree
    leafs_.clear();
    root_->findActiveLeafs(leafs_);

    normalizeProbabilities();

    ROS_DEBUG("   #leafs after pruning = %i\n", (int)leafs_.size());
//     printf("   #leafs after pruning = %i\n", (int)leafs_.size());

    ROS_DEBUG("pruneTree - end\n");
//      printf("pruneTree - end\n");
}

/* ****************************************************************************** */
/* *                                GETTERS                                     * */
/* ****************************************************************************** */

const std::list<Hypothesis*>& HypothesisTree::getHypotheses() const {
    return leafs_;
}

int HypothesisTree::getHeight() const {
    return tree_height_;
}


/*const Hypothesis& HypothesisTree::getMAPHypothesis() const {       
        std::cout << "HypothesisTree::getMAPHypothesis(): *MAP_hypothesis_ = " << MAP_hypothesis_ << std::endl;
    return *MAP_hypothesis_;
}*/

const Hypothesis& HypothesisTree::getMAPHypothesis() const 
{       
   double maxProb = 0.0;
   std::list<mhf::Hypothesis* >::const_iterator bestHyp;
         //const std::list<mhf::Hypothesis*>& hyp_list = hypothesisTree_->getHypotheses();
    for(std::list<mhf::Hypothesis* >::const_iterator it_hyp = leafs_.begin(); it_hyp != leafs_.end(); ++it_hyp) 
    {

      //  const std::list<mhf::SemanticObject*>* objs = (*it_hyp)->getObjects();

        double hypProb = (*it_hyp)->getProbability();
        if( hypProb > maxProb )
        {
                maxProb = hypProb;
                bestHyp = it_hyp;
        }
        
        
       // sumProb += hyp_prob;
       // std::cout << "Hyp P = " << hyp_prob << ": " << objs->size() << " object(s)" << std::endl;

    }
    
//    std::cout << "getMAPHypothesis best hyp =  " << *bestHyp << std::endl;
    
    return **bestHyp;
    //std::cout << "totalProb = " << sumProb << std::endl;
}

const std::list<std::shared_ptr<SemanticObject>>* HypothesisTree::getMAPObjects() const {
    ROS_DEBUG("getMAPObjects - begin\n");
//     std::cout << "getMAPObjects - begin\n " << std::endl;
    return getMAPHypothesis().getObjects();
}


/* ****************************************************************************** */
/* *                              PRINT METHODS                                 * */
/* ****************************************************************************** */

void HypothesisTree::showStatistics() {
    std::cout << "   Number of hypotheses        = " << leafs_.size() << std::endl;
    std::cout << "   Max probability             = " << getMAPHypothesis().getProbability() << std::endl;
    std::cout << "   Tree height                  = " << tree_height_ << std::endl;
}

}
