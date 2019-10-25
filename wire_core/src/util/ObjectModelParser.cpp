/*
 * ObjectModelParser.cpp
 *
 *  Created on: Mar 6, 2012
 *      Author: sdries
 */

#include "wire/util/ObjectModelParser.h"
#include "wire/core/ClassModel.h"
#include "wire/storage/KnowledgeDatabase.h"

#include <problib/pdfs/Uniform.h>
#include <problib/pdfs/PMF.h>

#define NO_FILE "None"

using namespace std;

namespace mhf {


ObjectModelParser::ObjectModelParser() : object_model_loader_(new pluginlib::ClassLoader<IStateEstimator>("wire_core", "IStateEstimator")){
}
        
ObjectModelParser::ObjectModelParser(const std::string& filename) : filename_(filename), 
    object_model_loader_(new pluginlib::ClassLoader<IStateEstimator>("wire_core", "IStateEstimator")) {
//std::cout << "Constructor: filename_ = " << filename_ << std::endl;
}

ObjectModelParser::ObjectModelParser(const tue::Configuration& config) : filename_(NO_FILE), config_(config),
    object_model_loader_(new pluginlib::ClassLoader<IStateEstimator>("wire_core", "IStateEstimator")) {    
}

void ObjectModelParser::configure(const tue::Configuration& config)
{
        filename_ = NO_FILE;
        config_ = config;
}

void ObjectModelParser::configure(const std::string& filename)
{
        filename_ = filename;
}

ObjectModelParser::~ObjectModelParser() {
    // TODO: change ownership of the class_loader, as it must persist throughout the whole lifetime of the world model
    //delete object_model_loader_;
}

std::string ObjectModelParser::getErrorMessage() const {
    return parse_errors_.str();
}

string ObjectModelParser::getPropertyValue(const TiXmlElement* elem, string prop_name, double& value, stringstream& error, bool optional) {
    const TiXmlElement* p = elem->FirstChildElement(prop_name);
    if (p) {
        return p->Attribute("value", &value);
    }
    if (!optional) {
        error << "Could not find property '" << prop_name << "' of element '" << elem->Value() << "'" << endl;
    }
    return "";
}



bool ObjectModelParser::getAttributeValue(string att_name, string& att_value) {

    std::string value;
    config_.value(att_name.c_str(), value);
    
    att_value = value;
    return true;
}

bool ObjectModelParser::getAttributeValue(const TiXmlElement* elem, string att_name, string& att_value, stringstream& error) {
    if (!elem) return false;

    const char* value = elem->Attribute(att_name.c_str());
    if (!value) {
        error << "Could not find attribute '" << att_name << "' of element '" << elem->Value() << "'" << endl;
        return false;
    }

    att_value = value;
    return true;
}

bool ObjectModelParser::getAttributeValue(string att_name, double& att_value) {
    config_.value(att_name.c_str(), att_value);
    return true;
}


bool ObjectModelParser::getAttributeValue(const TiXmlElement* elem, string att_name, double& att_value, stringstream& error) {
    if (!elem) return false;

    const char* value = elem->Attribute(att_name.c_str(), &att_value);

    if (!value) {
        error << "Could not find attribute '" << att_name << "' of element '" << elem->Value() << "'" << endl;
        return false;
    }

    return true;
}

bool ObjectModelParser::hasAttributeValue(const TiXmlElement* elem, string att_name, string att_value) {
    if (!elem) return false;

    const char* value = elem->Attribute(att_name.c_str());
    if (!value) return false;

    string valueStr = value;
    return (att_value == valueStr);
}

std::shared_ptr<pbl::PDF> ObjectModelParser::parsePDF(const TiXmlElement* pdf_elem, std::stringstream& error) {
    const char* pdf_type = pdf_elem->Attribute("type");
    if (pdf_type) {
        if (string(pdf_type) == "uniform") {
            double dim = 0;
            double density = 0;
            if (getAttributeValue(pdf_elem, "dimensions", dim, error)
                    && getAttributeValue(pdf_elem, "density", density, error)) {
                return std::make_shared<pbl::Uniform>((int)dim, density);
            }
        } else if (string(pdf_type) == "discrete") {
            double domain_size;
            if (getAttributeValue(pdf_elem, "domain_size", domain_size, error)) {
                return std::make_shared<pbl::PMF>((int)domain_size);
            }
        } else {
            error << "Unknown pdf type: " << pdf_type << endl;
        }
    } else {
        error << "PDF specification should contain 'type' attribute" << endl;
    }
    return 0;
}


bool ObjectModelParser::parseStateEstimator(ClassModel* obj_model, const TiXmlElement* elem, std::stringstream& error) {

    // check behavior model's attribute and model type
    string attribute_name, model_type;
    if (!getAttributeValue(elem, "attribute", attribute_name, error)
            | !getAttributeValue(elem, "model", model_type, error)) {
        return false;
    }

    Attribute attribute = AttributeConv::attribute(attribute_name);

    if (!object_model_loader_->isClassAvailable(model_type)){
        std::vector<std::string> classes = object_model_loader_->getDeclaredClasses();
        for(unsigned int i = 0; i < classes.size(); ++i){
            if(model_type == object_model_loader_->getName(classes[i])){
                //if we've found a match... we'll get the fully qualified name and break out of the loop
                ROS_WARN("Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                        model_type.c_str(), classes[i].c_str());
                model_type = classes[i];
                break;
            }
        }
    }

    IStateEstimator* estimator;

    if (object_model_loader_->isClassAvailable(model_type)) {
        estimator = object_model_loader_->createClassInstance(model_type)->clone();
    } else {
        error << "Unknown model: " << model_type << endl;
        return false;
    }

    // set estimator parameters
    const TiXmlElement* param = elem->FirstChildElement("param");
    while (param) {
        const char* param_name = param->Attribute("name");

        if (param_name) {
            bool v_bool;
            int v_int;
            double v_double;

            bool set_param_ok = true;
            if (param->QueryDoubleAttribute("value", &v_double) == TIXML_SUCCESS) {
                set_param_ok = estimator->setParameter(string(param_name), v_double);
            } else if (param->QueryIntAttribute("value", &v_int) == TIXML_SUCCESS) {
                set_param_ok = estimator->setParameter(string(param_name), (double)v_int);
            } else if (param->QueryBoolAttribute("value", &v_bool) == TIXML_SUCCESS) {
                set_param_ok = estimator->setParameter(string(param_name), v_bool);
            } else {
                const char* v_str = param->Attribute("value");
                if (v_str) {
                    set_param_ok = estimator->setParameter(string(param_name), string(v_str));
                } else {
                    error << "State estimator parameters should always have a 'name' and 'value' attribute." << endl;
                }
            }

            if (!set_param_ok) {
                error << "Unknown parameter for estimator '" << model_type << "': " << param_name << endl;
            }

        } else {
            error << "State estimator parameters should always have a 'name' and 'value' attribute." << endl;
        }

        param = param->NextSiblingElement("param");
    }

    const TiXmlElement* pnew = elem->FirstChildElement("pnew");
    if (pnew) {
        std::shared_ptr<pbl::PDF> pdf_new = parsePDF(pnew, error);
        if (pdf_new) {
            obj_model->setNewPDF(attribute, *pdf_new);

            estimator->update(pdf_new, 0);

            //delete pdf_new;
        } else {
            return false;
        }
    } else {
        error << "Estimator specification does not contain 'pnew'." << endl;
        return false;
    }

    const TiXmlElement* pclutter = elem->FirstChildElement("pclutter");
    if (pclutter) {
        std::shared_ptr<pbl::PDF> pdf_clutter = parsePDF(pclutter, error);
        if (pdf_clutter) {
            obj_model->setClutterPDF(attribute, *pdf_clutter);

            //delete pdf_clutter;
        } else {
            return false;
        }
    } else {
        error << "Estimator specification does not contain 'pclutter'." << endl;
        return false;
    }

    obj_model->setEstimator(attribute, *estimator);

    return true;
}

bool ObjectModelParser::getStateEstimatorParameter(const TiXmlElement* elem, const string& param_name, double& value) {

    const TiXmlElement* param = elem->FirstChildElement("param");
    while (param) {
        const char* v = param->Attribute("name");
        if (v && (string)v == param_name) {
            param->Attribute("value", &value);
            return true;
        }
        param = param->NextSiblingElement("param");
    }

    return false;
}


bool ObjectModelParser::parse(KnowledgeDatabase& knowledge_db) 
{
 std::cout <<  "Going to parse" << std::endl;  
 bool parseSucceeded;
        if(filename_ == NO_FILE) // yaml version
        {
                parseSucceeded = parseYAML(knowledge_db);
        }
        else // xml version
        {
                parseSucceeded = parseXML(knowledge_db);
        }

   // std::cout << "Info in knowledge_db = \n";
   // std::cout << knowledge_db.classModelsToString();
    
    return parseSucceeded;
    
}

bool ObjectModelParser::parseXML(KnowledgeDatabase& knowledge_db) {

    TiXmlDocument doc(filename_);
    doc.LoadFile();

    if (doc.Error()) {
        ROS_ERROR_STREAM("While parsing '" << filename_ << "': " << endl << endl << doc.ErrorDesc() << " at line " << doc.ErrorRow() << ", col " << doc.ErrorCol());
        return false;
    }

    const TiXmlElement* root = doc.RootElement();

    double prior_new;
    const TiXmlElement* prior_new_elem = root->FirstChildElement("prior_new");
    if (prior_new_elem) {
        if (getAttributeValue(prior_new_elem, "value", prior_new, parse_errors_)) {
            knowledge_db.setPriorNew(prior_new);
        }
    } else {
        parse_errors_ << "Knowledge file does not contain 'prior_new'" << endl;
    }

    double prior_existing;
    const TiXmlElement* prior_existing_elem = root->FirstChildElement("prior_existing");
    if (prior_existing_elem) {
        if (getAttributeValue(prior_existing_elem, "value", prior_existing, parse_errors_)) {
            knowledge_db.setPriorExisting(prior_existing);
        }
    } else {
        parse_errors_ << "Knowledge file does not contain 'prior_existing'" << endl;
    }

    double prior_clutter;
    const TiXmlElement* prior_clutter_elem = root->FirstChildElement("prior_clutter");
    if (prior_clutter_elem) {
        if (getAttributeValue(prior_clutter_elem, "value", prior_clutter, parse_errors_)) {
            knowledge_db.setPriorClutter(prior_clutter);
        }
    } else {
        parse_errors_ << "Knowledge file does not contain 'prior_clutter'" << endl;
    }


    const TiXmlElement* class_element = root->FirstChildElement("object_class");

    /* PARSE ALL OBJECT MODELS */

    while(class_element) {
        ClassModel* class_model = 0;

        string model_name;
        getAttributeValue(class_element, "name", model_name, parse_errors_);

        cout << "Parsing model for class " << model_name << endl;

        string base_class = "";
        const char* value = class_element->Attribute("base");

        if (value) {
            // class derives from base class
            base_class = value;
            
            std::cout << "base_class = " << base_class << std::endl;

            const ClassModel* base_model = knowledge_db.getClassModel(base_class);
            if (base_model) {
                class_model = new ClassModel(*base_model);
                class_model->setModelName(model_name);
            } else {
                parse_errors_ << "Error in class definition of '" << model_name << "': unknown base class '" << base_class << "'." << endl;
                class_model = new ClassModel(model_name);
            }
        } else {
            class_model = new ClassModel(model_name);
        }

        // parse properties
        const TiXmlElement* prop = class_element->FirstChildElement();
        while(prop) {
            string prop_name = prop->Value();
            if (prop_name == "behavior_model") {
                stringstream bh_errors;
                parseStateEstimator(class_model, prop, bh_errors);
                if (bh_errors.str() != "") {
                    parse_errors_ << "In class description for '" << class_model->getModelName() << "': " << bh_errors.str() << endl;
                }
            } else {
                parse_errors_ << "In class description for '" << class_model->getModelName() << "': Unknown class property: '" << prop_name << "'" << endl;
            }
            prop = prop->NextSiblingElement();
        }

        knowledge_db.addClassModel(class_model->getModelName(), class_model);

        class_element = class_element->NextSiblingElement("object_class");
    }

    if (parse_errors_.str() != "") {
        return false;
    }

    return true;
}

std::shared_ptr<pbl::PDF> ObjectModelParser::parsePDFYAML(std::stringstream& error) {
    std::string pdf_type;
    config_.value("type", pdf_type);

        if (string(pdf_type) == "uniform") {
            double dim = 0;
            double density = 0;
            if (getAttributeValue("dimensions", dim)
                    && getAttributeValue("density", density)) {
                return std::make_shared<pbl::Uniform>((int)dim, density);
            }
        } else if (string(pdf_type) == "discrete") {
            double domain_size;
            if (getAttributeValue("domain_size", domain_size)) {
                return std::make_shared<pbl::PMF>((int)domain_size);
            }
        } else {
            error << "Unknown pdf type: " << pdf_type << endl;
        }

    return 0;
}

bool ObjectModelParser::parseYAML(KnowledgeDatabase& knowledge_db) 
{
    if (config_.readGroup("knowledge", tue::REQUIRED))
    {            
        float prior_new = 0.0, prior_existing = 0.0, prior_clutter = 0.0; // todo: pass to the corresponding place
        config_.value("prior_new", prior_new);
        config_.value("prior_existing", prior_existing);
        config_.value("prior_clutter", prior_clutter);
         
        knowledge_db.setPriorNew(prior_new);
        knowledge_db.setPriorExisting(prior_existing);
        knowledge_db.setPriorClutter(prior_clutter);
        
        std::cout << "prior_new = " << prior_new <<
                  "\nprior_existing = " <<  prior_existing << 
                  "\nprior_clutter = " <<  prior_clutter << std::endl;

        if(config_.readArray("object_class", tue::REQUIRED)) 
        {                
                while(config_.nextArrayItem())
                {
                        /* PARSE ALL OBJECT MODELS */
                         ClassModel* class_model = 0;
                         
                        std::string objectClassName, baseName;
                        config_.value("name", objectClassName, tue::REQUIRED);
                        
                        std::cout << "\nClass name = " << objectClassName << std::endl;
                        if( config_.value("base", baseName, tue::OPTIONAL) )
                        {
                                 std::cout << "base_class = " << baseName << std::endl;
                                 const ClassModel* base_model = knowledge_db.getClassModel(baseName);
                                 
                                 if (base_model) 
                                 {
                                         class_model = new ClassModel(*base_model);
                                         class_model->setModelName(objectClassName);
                                 }
                                 else 
                                 {
                                         ROS_WARN( "Error in class definition of '%s': unknown base class '%s'.\n", objectClassName.c_str(), baseName.c_str());
                                         class_model = new ClassModel(objectClassName);
                                 }
                        } 
                        else
                        {
                                class_model = new ClassModel(objectClassName);
                        }
                        
                        if (config_.readArray("behavior_model"))
                        {
                                std::cout << "Attributes:" << std::endl;
                                while(config_.nextArrayItem())
                                {
                                        std::string attribute_name, model_type;
                                        config_.value("attribute", attribute_name);
                                        config_.value("model", model_type);
                                        std::cout << "\tFor attribute '" << attribute_name << 
                                        "'\n\tmodel = " << model_type << std::endl;

                                        Attribute attribute = AttributeConv::attribute(attribute_name);
                                        
                                        if (!object_model_loader_->isClassAvailable(model_type))
                                        {
                                                std::vector<std::string> classes = object_model_loader_->getDeclaredClasses();
                                                for(unsigned int i = 0; i < classes.size(); ++i)
                                                {
                                                        if(model_type == object_model_loader_->getName(classes[i]))
                                                        {
                                                                //if we've found a match... we'll get the fully qualified name and break out of the loop
                                                                ROS_WARN("Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                                                                        model_type.c_str(), classes[i].c_str());
                                                                model_type = classes[i];
                                                                break;
                                                        }
                                                }
                                        }
                                                                
                                        IStateEstimator* estimator;

                                        if (object_model_loader_->isClassAvailable(model_type)) 
                                        {
                                                estimator = object_model_loader_->createClassInstance(model_type)->clone();
                                        } 
                                        else
                                        {
                                                std::cout << "Unknown model: " << model_type << endl;
                                                return false;
                                        }
                                        
                                        if(config_.readGroup("pnew", tue::REQUIRED))
                                        {
                                                std::stringstream errorTest;
                                                std::shared_ptr<pbl::PDF> pdf_new = parsePDFYAML( errorTest);
                                                
                                                class_model->setNewPDF(attribute, *pdf_new);
                                                estimator->update(pdf_new, 0);
        
                                                std::cout << "\tpnew = " << pdf_new->toString() << std::endl;
                                                config_.endGroup(); // pnew  
                                        }
                                        
                                        if(config_.readGroup("pclutter", tue::REQUIRED))
                                        {
                                                std::stringstream errorTest;
                                                std::shared_ptr<pbl::PDF> pdf_new = parsePDFYAML( errorTest);
                                                class_model->setClutterPDF(attribute, *pdf_new);
                                                
                                                std::cout << "\tpclutter = " << pdf_new->toString() << std::endl;
                                                config_.endGroup(); // pclutter
                                        }
                                        
                                        if (config_.readArray("parameters"))
                                        {
                                                std::cout << "\tparameters:" << std::endl;
                                                while(config_.nextArrayItem())
                                                {
                                                        std::string name; // TODO parameters for different types?! (bool and int still remain)
                                                        double value;
                                                        config_.value("name", name);
                                                        config_.value("value", value);
                                                        
                                                        std::cout << "\t\tName = " << name  << " value: " << value << std::endl;
                                                        estimator->setParameter(name, value);
                                                }
                                                config_.endArray(); // parameters
                                        }
                                        
                                        class_model->setEstimator(attribute, *estimator);
                                }
                                
                                config_.endArray(); // behavior_model
                        }
                        
                        knowledge_db.addClassModel(class_model->getModelName(), class_model);                        
                 }
                config_.endArray(); // object_class
        }
        
        config_.endGroup(); // knowledge
    }
    
    return true;
}

}
