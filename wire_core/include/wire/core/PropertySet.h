#ifndef PROPERTY_SET_H_
#define PROPERTY_SET_H_

#include "wire/core/datatypes.h"
#include "wire/core/IStateEstimator.h"

#include "problib/pdfs/PDF.h"

#define OBJECT_TIMEOUT_TIME 3 // TODO make configurable

namespace mhf {

class Property;

class PropertySet : public IStateEstimator {
public:

    static int N_PROPERTY_SET;

    PropertySet(Time timestamp = 0);

    PropertySet(const PropertySet& orig);

    virtual ~PropertySet();

//     std::shared_ptr<PropertySet> clone() const;
    virtual std::shared_ptr<IStateEstimator> clone() const { return cloneThis(); };
  
    std::shared_ptr<PropertySet> cloneThis() const 
    {             
            return std::make_shared< PropertySet>(*this);
    }

    void addProperty(const Attribute& attribute, const pbl::PDF& value);

    void addProperty(const std::string& attribute, const pbl::PDF& value);

    void addProperty(const Attribute& attribute, const IStateEstimator& estimator);    

    std::shared_ptr<const Property> getProperty(const Attribute& attribute) const;

    std::shared_ptr<const Property> getProperty(const std::string& attribute) const;

    void propagate(const Time& time);

    void update(std::shared_ptr<const pbl::PDF> z, const Time& time);

    void reset();

    std::shared_ptr<const pbl::PDF> getValue() const;
    
    std::shared_ptr<const pbl::PDF> getFullValue() const;

    virtual double getLikelihood(const PropertySet& P) const;

    const std::map<Attribute, std::shared_ptr<Property>>& getPropertyMap() const;

    Time getTimestamp() const;
    
    Time getLatestUpdateTime () ;

    std::string toString() const;

protected:

    Time timestamp_;

    void addProperty(std::shared_ptr<Property> property);

    std::shared_ptr<Property> getProperty(const Attribute& attribute);

private:

    std::map<Attribute, std::shared_ptr<Property>> properties_;

};

}

#endif /* PROPERTY_SET_H_ */
