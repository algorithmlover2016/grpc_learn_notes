#pragma once
#include <string>

class Resource {
private:
    std::string name;
public:
    Resource(std::string n);

    Resource(Resource const & r);

    Resource& operator= (Resource const & r);

    Resource(Resource&& r);

    ~Resource(void);

    std::string GetName() const { return name;}
};