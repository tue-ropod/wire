#ifndef PROPERTY_SET_H_
#define PROPERTY_SET_H_

#include "wire/core/datatypes.h"
#include "wire/core/IStateEstimator.h"

#include "problib/pdfs/PDF.h"

namespace mhf {

class Property;

class PropertySet : public IStateEstimator {
public:

    //static int N_PROPERTY_SET;

    PropertySet(Time timestamp = 0);

    PropertySet(std::shared_ptr<const PropertySet> orig);

    virtual ~PropertySet();

    //std::shared_ptr<PropertySet> clone() const;
    
     std::shared_ptr<IStateEstimator> clone() const{ return CloneMethod(); };
    
    std::shared_ptr<PropertySet> CloneMethod() const { 
            std::shared_ptr<PropertySet> PS = std::make_shared< PropertySet>(*this);
            
            return PS;
}

    void addProperty(const Attribute& attribute, std::shared_ptr<const pbl::PDF> value);

    void addProperty(const std::string& attribute, std::shared_ptr<const pbl::PDF> value);

    void addProperty(const Attribute& attribute, std::shared_ptr<const IStateEstimator> estimator);    

    const std::shared_ptr<  Property> getProperty(const Attribute& attribute) const;

    const std::shared_ptr< Property> getProperty(const std::string& attribute) const;

    void propagate(const Time& time);

    void update(std::shared_ptr<const pbl::PDF> z, const Time& time);

    void reset();

    std::shared_ptr<const pbl::PDF> getValue() const;
    //const pbl::PDF& getValue() const;

    virtual double getLikelihood(const std::shared_ptr< PropertySet> P) const;

    const std::map<Attribute, std::shared_ptr<Property>>& getPropertyMap() const;

    Time getTimestamp() const;

    std::string toString() const;

protected:

    Time timestamp_;

    void addProperty(std::shared_ptr<Property> property);

    //std::shared_ptr<Property> getProperty(const Attribute& attribute);

private:

    std::map<Attribute, std::shared_ptr<Property>> properties_;

};

}

#endif /* PROPERTY_SET_H_ */
