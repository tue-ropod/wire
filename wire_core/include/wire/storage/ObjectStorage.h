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

    void addObject(SemanticObject* obj);

    std::list<SemanticObject*>::iterator removeObject(SemanticObject& obj);

    long getUniqueID();

    void match(const Evidence& ev);
    
   std::shared_ptr< std::list<SemanticObject*> > getObjects() const;

protected:

    ObjectStorage();

    static ObjectStorage* instance_;


    long ID_;

    std::shared_ptr<std::list<SemanticObject*>> objects_;

    const KnowledgeDatabase& knowledge_db_;   

};

}

#endif
