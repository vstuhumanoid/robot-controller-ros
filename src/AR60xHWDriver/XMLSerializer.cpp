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

    XMLElement *connection = 0;
    XMLElement *joints = 0;
    XMLElement *sensors = 0;
    XMLElement *jointData = 0;
    XMLElement *sensorData = 0;

    connection = root->FirstChildElement("connection");

    ConnectionData connectionData;
    const char* host = connection->Attribute("host");
    connectionData.host = std::string(host);
    connection->QueryAttribute("recvPort", &connectionData.recvPort);
    connection->QueryAttribute("send_delay_", &connectionData.sendDelay);
    connection->QueryAttribute("sendPort", &connectionData.sendPort);

    joints = root->FirstChildElement("joints");
    jointData = joints->FirstChildElement("joint");

    std::map <int, JointData> jointsMap;

    while(jointData != nullptr)
    {
        //int number;
        //eResult = jointData->QueryAttribute("number", &number);
        //if(eResult != XML_SUCCESS) return;

        JointData joint;
        jointData->QueryAttribute("number", &joint.number);
        const char* name = jointData->Attribute("name");
        joint.name = std::string(name);
        jointData->QueryAttribute("channel", &joint.channel);

        XMLElement *jointGains = jointData->FirstChildElement("gains");

        jointGains->QueryAttribute("proportional", &joint.gains.proportional);
        jointGains->QueryAttribute("integral", &joint.gains.integral);
        jointGains->QueryAttribute("derivative", &joint.gains.derivative);

        XMLElement *jointLimits = jointData->FirstChildElement("limits");

        jointLimits->QueryAttribute("lowerLimit", &joint.limits.lowerLimit);
        jointLimits->QueryAttribute("upperLimit", &joint.limits.upperLimit);

        jointData->QueryAttribute("offset", &joint.offset);
        jointData->QueryAttribute("isReverce", &joint.isReverce);
        jointData->QueryAttribute("isEnable", &joint.isEnable);

        jointsMap[joint.number] = joint;

        jointData = jointData->NextSiblingElement("joint");
    }

    sensors = root->FirstChildElement("sensors");
    sensorData = sensors->FirstChildElement("sensor");

    std::map <int, SensorData> sensorsMap;

    while(sensorData != nullptr)
    {
        SensorData sensor;
        sensorData->QueryAttribute("number", &sensor.number);
        const char* name = sensorData->Attribute("name");
        sensor.name = std::string(name);
        sensorData->QueryAttribute("channel", &sensor.channel);

        sensorsMap[sensor.number] = sensor;

        sensorData = sensorData->NextSiblingElement("sensor");
    }

    desc->joints = jointsMap;
    desc->sensors = sensorsMap;
    *conn = connectionData;

    return true;
}

bool XMLSerializer::serialize(std::string fileName, AR60xDescription * desc, ConnectionData * conn)
{
    XMLDocument document;
    XMLNode * root = document.NewElement("config");

    XMLElement *connectionData = document.NewElement("connection");
    connectionData->SetAttribute("host", conn->host.c_str() );
    connectionData->SetAttribute("recvPort", conn->recvPort);
    connectionData->SetAttribute("send_delay_", conn->sendDelay);
    connectionData->SetAttribute("sendPort", conn->sendPort);
    root->InsertEndChild(connectionData);

    XMLElement *joints = document.NewElement("joints");

    for(auto it = desc->joints.begin(); it != desc->joints.end(); ++it)
    {
        JointData joint = (*it).second;

        XMLElement *jointData = document.NewElement("joint");
        jointData->SetAttribute("number", joint.number);
        jointData->SetAttribute("name", joint.name.c_str() );
        jointData->SetAttribute("channel", joint.channel);

        XMLElement *jointGains = document.NewElement("gains");
        jointGains->SetAttribute("proportional", joint.gains.proportional);
        jointGains->SetAttribute("integral", joint.gains.integral);
        jointGains->SetAttribute("derivative", joint.gains.derivative);
        jointData->InsertEndChild(jointGains);

        XMLElement *jointLimits = document.NewElement("limits");
        jointLimits->SetAttribute("lowerLimit", joint.limits.lowerLimit);
        jointLimits->SetAttribute("upperLimit", joint.limits.upperLimit);
        jointData->InsertEndChild(jointLimits);

        jointData->SetAttribute("offset", joint.offset);
        jointData->SetAttribute("isReverce", joint.isReverce);
        jointData->SetAttribute("isEnable", joint.isEnable);

        joints->InsertEndChild(jointData);
    }

    root->InsertEndChild(joints);
    XMLElement *sensors = document.NewElement("sensors");

    for(auto it = desc->sensors.begin(); it != desc->sensors.end(); ++it)
    {
        SensorData sensor = (*it).second;

        XMLElement *sensorData = document.NewElement("sensor");
        sensorData->SetAttribute("number", sensor.number);
        sensorData->SetAttribute("name", sensor.name.c_str() );
        sensorData->SetAttribute("channel", sensor.channel);

        sensors->InsertEndChild(sensorData);
    }

    root->InsertEndChild(sensors);
    document.InsertEndChild(root);

    XMLError eResult = document.SaveFile(fileName.c_str());

    if(eResult != XML_SUCCESS)
    {
        return false;
    }

    return true;
}
