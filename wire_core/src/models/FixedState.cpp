/*
 * FixedState.cpp
 *
 *  Created on: Jul 3, 2012
 *      Author: sdries
 */

#include "wire/models/FixedState.h"

namespace mhf {

FixedState::FixedState() {

}

FixedState::FixedState(std::shared_ptr<const pbl::PDF> pdf) : pdf_(pdf->clone()) {
}

FixedState::FixedState(std::shared_ptr<const FixedState> orig) : IStateEstimator(*orig), pdf_(orig->pdf_->clone()) {
}

FixedState::~FixedState() {
  //  delete pdf_;
}

//std::shared_ptr<FixedState> FixedState::clone() const {
//    //return new FixedState(*this);
//        return std::make_shared<FixedState>(shared_from_this());
//}

void FixedState::update(std::shared_ptr<const pbl::PDF> z, const mhf::Time& time) {
}

void FixedState::propagate(const mhf::Time& time) {
}

void FixedState::reset() {
}

std::shared_ptr<const pbl::PDF> FixedState::getValue() const {
    return pdf_;
}


/*std::shared_ptr<const pbl::PDF>& FixedState::getValue() const {
     std::shared_ptr<const pbl::PDF>& toReturn = std::make_shared<const pbl::PDF> (pdf_)  ;
    return pdf_;
}*/

}
