/*
 * FixedState.h
 *
 *  Created on: Jul 3, 2012
 *      Author: sdries
 */

#ifndef FIXEDSTATE_H_
#define FIXEDSTATE_H_

#include "wire/core/IStateEstimator.h"

namespace mhf {

/**
 * @brief FixedState is a simple wrapper such that probability density function can
 * be threaded as state estimator, i.e., FixedState reflects a state but update,
 * propagation and reset do not influence the state.
 */
//public std::enable_shared_from_this<FixedState>
class FixedState : public mhf::IStateEstimator   {

public:

    FixedState();

    FixedState(std::shared_ptr<const pbl::PDF> pdf);

    FixedState(std::shared_ptr<const FixedState> orig);

    virtual ~FixedState();

    //std::shared_ptr<IStateEstimator> clone() const;
    
    std::shared_ptr<IStateEstimator> clone() const{ return CloneMethod(); };
   
    std::shared_ptr<FixedState> CloneMethod() const {return std::make_shared< FixedState>(*this);}

    /**
     * @brief Performs an update, but since the state is fixed, update will do nothing.
     */
    virtual void update(std::shared_ptr<const pbl::PDF> z, const mhf::Time& time);

    /**
     * @brief Propagates the state, but since the state is fixed, propagate will do nothing.
     */

    virtual void propagate(const mhf::Time& time);

    /**
     * @brief Resets the state, but since the state is fixed, reset will do nothing.
     */
    virtual void reset();

    std::shared_ptr<const pbl::PDF> getValue() const;
     //std::shared_ptr<const pbl::PDF>& getValue() const;

protected:

    std::shared_ptr<pbl::PDF> pdf_;

};

}

#endif /* FIXEDSTATE_H_ */
