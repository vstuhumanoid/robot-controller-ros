#include "XMLSerializer.h"

XMLSerializer::XMLSerializer()
{

}

bool XMLSerializer::deserialize(std::string fileName, AR60xDescription * desc, ConnectionData * conn)
{
    XMLDocument document;
    XMLError eResult =  document.LoadFile(fileName.c_str());
    if(eResult != XML_SUCCESS)
    {
        return false;
    }

    XMLElement *root = document.FirstChildElement();
    XMLElement* connection = root->FirstChildElement("connection");

    ConnectionData connectionData;
    const char* host = connection->Attribute("host");
    connectionData.host = std::string(host);
    connection->QueryAttribute("robotPort", &connectionData.robotPort);
    connection->QueryAttribute("sendDelay", &connectionData.sendDelay);
    connection->QueryAttribute("localPort", &connectionData.localPort);

    XMLElement* joints = root->FirstChildElement("joints");
    XMLElement* jointData = joints->FirstChildElement("joint");

    std::map <int, JointData> jointsMap;

    while(jointData != nullptr)
    {
        // FUCK TinyXML BECAUSE IN DOESN'T SUPPORT STDINT

        JointData joint;

        int number;
        jointData->QueryAttribute("number", &number);
        joint.number = number;

        joint.description = std::string(jointData->Attribute("description")); //TODO: Does it allocate string inside parser?

        int channel;
        jointData->QueryAttribute("channel", &channel);
        joint.channel = channel;

        XMLElement *jointGains = jointData->FirstChildElement("gains");
        int gain;
        jointGains->QueryAttribute("proportional", &gain);
        joint.pid_gains.p = gain;
        jointGains->QueryAttribute("integral", &gain);
        joint.pid_gains.i = gain;
        jointGains->QueryAttribute("derivative", &gain);
        joint.pid_gains.d = gain;

        XMLElement *jointLimits = jointData->FirstChildElement("limits");

        jointLimits->QueryAttribute("lowerLimit", &joint.lower_limit);
        jointLimits->QueryAttribute("upperLimit", &joint.upper_limit);

        jointData->QueryAttribute("offset", &joint.offset);
        jointData->QueryAttribute("isReverse", &joint.is_reverse);
        jointData->QueryAttribute("isEnable", &joint.is_enable);

        jointsMap[joint.number] = joint;

        jointData = jointData->NextSiblingElement("joint");
    }

    XMLElement* sensorsGroupsData = root->FirstChildElement("sensorGroups");
    XMLElement* sensorsGroupData = sensorsGroupsData->FirstChildElement("group");

    std::map <int, SensorsGroupData> sensorsMap;

    while(sensorsGroupData != nullptr)
    {
        SensorsGroupData sensorsGroup;
        sensorsGroupData->QueryAttribute("id", &sensorsGroup.id);
        sensorsGroup.name = sensorsGroupData->Attribute("name");
        int channel;
        sensorsGroupData->QueryAttribute("channel", &channel);
        sensorsGroup.channel = channel;

        XMLElement* sensorData = sensorsGroupData->FirstChildElement("sensor");
        while(sensorData != nullptr)
        {
            SensorData sensor;
            sensor.name = sensorData->Attribute("name");
            int number;
            sensorData->QueryAttribute("number", &number);
            sensor.number = number;
            sensorData->QueryAttribute("offset", &sensor.offset);

            sensorsGroup.sensors.push_back(sensor);
            sensorData = sensorData->NextSiblingElement("sensor");
        }

        sensorsMap[sensorsGroup.id] = sensorsGroup;
        sensorsGroupData = sensorsGroupData->NextSiblingElement("group");
    }

    desc->joints = jointsMap;
    desc->sensorGroups = sensorsMap;
    *conn = connectionData;

    return true;
}

bool XMLSerializer::serialize(std::string fileName, AR60xDescription * desc, ConnectionData * conn)
{
    XMLDocument document;
    XMLNode * root = document.NewElement("config");

    XMLElement *connectionData = document.NewElement("connection");
    connectionData->SetAttribute("host", conn->host.c_str() );
    connectionData->SetAttribute("robotPort", conn->robotPort);
    connectionData->SetAttribute("send_delay_", conn->sendDelay);
    connectionData->SetAttribute("localPort", conn->localPort);
    root->InsertEndChild(connectionData);

    XMLElement *joints = document.NewElement("joints");

    //for(auto it = desc->joints.begin(); it != desc->joints.end(); ++it)
    for(auto j: desc->joints)
    {
        JointData joint = j.second;

        XMLElement *jointData = document.NewElement("joint");
        jointData->SetAttribute("number", joint.number);
        jointData->SetAttribute("description", joint.description.c_str() );
        jointData->SetAttribute("channel", joint.channel);

        XMLElement *jointGains = document.NewElement("gains");
        jointGains->SetAttribute("proportional", joint.pid_gains.p);
        jointGains->SetAttribute("integral", joint.pid_gains.i);
        jointGains->SetAttribute("derivative", joint.pid_gains.d);
        jointData->InsertEndChild(jointGains);

        XMLElement *jointLimits = document.NewElement("limits");
        jointLimits->SetAttribute("lowerLimit", joint.lower_limit);
        jointLimits->SetAttribute("upperLimit", joint.upper_limit);
        jointData->InsertEndChild(jointLimits);

        jointData->SetAttribute("offset", joint.offset);
        jointData->SetAttribute("isReverse", joint.is_reverse);
        jointData->SetAttribute("isEnable", joint.is_enable);

        joints->InsertEndChild(jointData);
    }

    root->InsertEndChild(joints);
    XMLElement *sensorsGroups = document.NewElement("sensorGroups");

    for(auto group: desc->sensorGroups)
    {
        XMLElement *sensorGroup = document.NewElement("group");
        sensorGroup->SetAttribute("id", group.second.id);
        sensorGroup->SetAttribute("name", group.second.name.c_str());
        sensorGroup->SetAttribute("channel", group.second.channel);

        for(auto sensor: group.second.sensors)
        {
            XMLElement* sensorData = document.NewElement("sensor");
            sensorData->SetAttribute("number", sensor.number);
            sensorData->SetAttribute("name", sensor.name.c_str());
            sensorData->SetAttribute("offset", sensor.offset);
            sensorGroup->InsertEndChild(sensorData);
        }

        sensorsGroups->InsertEndChild(sensorGroup);
    }

    root->InsertEndChild(sensorsGroups);
    document.InsertEndChild(root);

    XMLError eResult = document.SaveFile(fileName.c_str());

    if(eResult != XML_SUCCESS)
    {
        return false;
    }

    return true;
}
