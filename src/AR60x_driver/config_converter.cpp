#include <XMLSerializer/XMLSerializer.h>
#include <iostream>
#include <ros/package.h>
#include <cmath>

using namespace std;

int main(int argc, char** argv)
{
    /*if(argc!=3)
    {
        cout << "Invalid arguments\n";
        cout << "Usage:\n";
        cout << "config_converter -to_old/-to_new <source> <dest>\n";
        return -1;
    }*/

    string old_path = ros::package::getPath("robot-controller-ros") + "/config.xml";
    string new_path = ros::package::getPath("robot-controller-ros") + "/config_new.xml";

    XMLSerializer serializer;

    AR60xDescription desc;
    ConnectionData conn;
    serializer.deserialize(old_path, &desc, &conn);

    for(auto& joint: desc.joints)
    {
        // FUCK C++
        joint.second.offset = round(joint.second.offset / 100 * 10000) / 10000;
        joint.second.limits.lowerLimit = round(joint.second.limits.lowerLimit / 100 * 10000) / 10000;
        joint.second.limits.upperLimit = round(joint.second.limits.upperLimit / 100 * 10000) / 10000;
    }

    serializer.serialize(new_path, &desc, &conn);
}