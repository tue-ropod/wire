/*
 * ObjectModelParser.h
 *
 *  Created on: Mar 6, 2012
 *      Author: sdries
 */

#ifndef OBJECTMODELPARSER_H_
#define OBJECTMODELPARSER_H_

#include "wire/core/IStateEstimator.h"

// xml parser
#include <tinyxml.h>

#include <pluginlib/class_loader.h>

//#include <tue/config/writer.h>
//#include <tue/config/loaders/yaml.h>
#include "tue/config/configuration.h"

#include <map>

namespace mhf {

class ClassModel;
class KnowledgeDatabase;

class ObjectModelParser {

public:

    ObjectModelParser();
        
    ObjectModelParser(const std::string& filename);
    
    ObjectModelParser(const tue::Configuration& config);
    
    void configure(const tue::Configuration& config);
    
    void configure(const std::string& filename);

    virtual ~ObjectModelParser();

    bool parse(KnowledgeDatabase& obj_models);
    
    bool parseXML(KnowledgeDatabase& knowledge_db);
    
    bool parseYAML(KnowledgeDatabase& knowledge_db);

    std::string getErrorMessage() const;

protected:

    std::string filename_;
    
    tue::Configuration config_;

    std::stringstream parse_errors_;

    pluginlib::ClassLoader<IStateEstimator>* object_model_loader_;

    std::string getPropertyValue(const TiXmlElement* elem, std::string prop_name, double& value, std::stringstream& error, bool optional = false);
    
    bool getAttributeValue(std::string att_name, std::string& att_value);

    bool getAttributeValue(const TiXmlElement* elem, std::string att_name, std::string& att_value, std::stringstream& error);
    
    bool getAttributeValue(std::string att_name, double& att_value);

    bool getAttributeValue(const TiXmlElement* elem, std::string att_name, double& att_value, std::stringstream& error);

    bool hasAttributeValue(const TiXmlElement* elem, std::string att_name, std::string att_value);

    bool parseStateEstimator(ClassModel* obj_model, const TiXmlElement* elem, std::stringstream& error);

    std::shared_ptr<pbl::PDF> parsePDF(const TiXmlElement* elem, std::stringstream& error);
    
    std::shared_ptr<pbl::PDF> parsePDFYAML(std::stringstream& error);

    bool getStateEstimatorParameter(const TiXmlElement* elem, const std::string& param_name, double& value);
    
    

};

}

#endif /* OBJECTMODELPARSER_H_ */
