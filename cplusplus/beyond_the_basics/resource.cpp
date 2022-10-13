#include "resource.h"
#include <iostream>

Resource::Resource(std::string n) : name(n) {
    std::cout << "constructing " << name << " done;\n";
}

Resource::Resource(Resource const & r) : name(r.name) {
    std::cout << "copy constructing " << name << "done;\n";
}

Resource& Resource::operator=(Resource const & r) {
    /*
    If this class managed Resource lifttime, clean up existing one
    before setting new values. No need here because string takes care of it.
    */
    name = r.GetName();
    std::cout << "copy assigning " << name << "done;\n";
    return *this;
}

Resource::~Resource(void) {
    std::cout << "desconstructing " << "done;\n";
}

Resource::Resource(Resource&& r) : name(r.name) {
    std::cout << "move constructing " << name << "done;\n";
}