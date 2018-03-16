#ifndef XMLSERIALIZER_H
#define XMLSERIALIZER_H

#include "Libs/TinyXML2/tinyxml2.h"
#include "RobotDescription/AR60xDescription.h"
#include "DataTypes/ConnectionData.h"
#include <string>

using namespace tinyxml2;

class XMLSerializer
{
public:
    XMLSerializer();

    bool serialize(std::string fileName, AR60xDescription * desc, ConnectionData * conn);
    bool deserialize(std::string fileName, AR60xDescription * desc, ConnectionData * conn);
};

#endif // XMLSERIALIZER_H
