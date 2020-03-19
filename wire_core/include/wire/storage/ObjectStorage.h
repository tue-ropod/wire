#ifndef MHT_OBJECT_STORAGE_H_
#define MHT_OBJECT_STORAGE_H_

#include <list>
#include <memory>

namespace mhf {

class SemanticObject;
class Evidence;
class KnowledgeDatabase;

class ObjectStorage {

public:

    static ObjectStorage& getInstance();

    virtual ~ObjectStorage();

    void addObject(std::shared_ptr<SemanticObject> obj);

    std::list<std::shared_ptr<SemanticObject>>::iterator removeObject(SemanticObject& obj);

    long getUniqueID();

    void match(const Evidence& ev);
    
   std::shared_ptr< std::list<std::shared_ptr<SemanticObject>> > getObjects() const;
   
   std::string allObjects2String() const;

protected:

    ObjectStorage();

    static ObjectStorage* instance_;


    long ID_;

    std::shared_ptr<std::list<std::shared_ptr<SemanticObject>>> objects_;

    const KnowledgeDatabase& knowledge_db_;   

};

}

#endif
