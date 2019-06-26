#ifndef MHT_OBJECT_STORAGE_H_
#define MHT_OBJECT_STORAGE_H_

#include <list>
#include <iostream> 
#include <memory>

namespace mhf {

class SemanticObject;
class Evidence;
class KnowledgeDatabase;

class ObjectStorage {

public:

    static std::shared_ptr<ObjectStorage> getInstance();

    virtual ~ObjectStorage();

    void addObject(std::shared_ptr<SemanticObject> obj);

    void removeObject(std::shared_ptr<SemanticObject> obj);

    long getUniqueID();

    void match(std::shared_ptr<const Evidence> ev);



    ObjectStorage(); // TODO protected??
    
    protected:

    static std::shared_ptr< ObjectStorage> instance_;


    long ID_;

    std::list<std::shared_ptr<SemanticObject>> objects_;

    std::shared_ptr<const KnowledgeDatabase> knowledge_db_;   

};

}

#endif
