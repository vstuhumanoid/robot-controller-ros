#include "AR60xRecvPacket.h"

AR60xRecvPacket::AR60xRecvPacket(AR60xDescription& robotDesc) : BasePacket(robotDesc)
{
}


void AR60xRecvPacket::initFromByteArray(const uint8_t *bytes)
{
    memcpy(byte_array_, bytes, packetSize);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



sensor_msgs::JointState AR60xRecvPacket::JointsGetState()
{
    sensor_msgs::JointState msg;
    msg.name.resize(desc_.joints.size());
    msg.position.resize(desc_.joints.size());

    int i = 0;
    for(auto& joint: desc_.joints)
    {
        int16_t value = read_int16(joint.second.channel * 16 + JointPositionAddress);
        msg.name[i] = std::to_string(joint.second.number);
        msg.position[i] = uint16_to_angle(value);
        i++;
    }

    return msg;
}

robot_controller_ros::JointsParams AR60xRecvPacket::JointsGetParams()
{
    robot_controller_ros::JointsParams msg;
    msg.names.resize(desc_.joints.size());
    msg.params.resize(desc_.joints.size());

    int i = 0;
    for(auto& joint: desc_.joints)
    {
        msg.names[i] = std::to_string(joint.second.number);
        msg.params[i].lower_limit  = jointGetLowerLimit(joint.second);
        msg.params[i].upper_limit  = jointGetUpperLimit(joint.second);
        msg.params[i].reverse = joint.second.isReverse;
        msg.params[i].mode = jointGetMode(joint.second);
        //TODO: PIDs
        //TODO: Enable
        i++;
    }

    return msg;
}


robot_controller_ros::JointsSupplyState AR60xRecvPacket::PowerGetJointsSupplyState()
{
    robot_controller_ros::JointsSupplyState msg;
    msg.names.resize(desc_.joints.size());
    msg.states.resize(desc_.joints.size());

    int i = 0;
    for(auto& joint: desc_.joints)
    {
        msg.names[i] = std::to_string(joint.second.number);
        msg.states[i] = jointGetSupplyState(joint.second);
        i++;
    }

    return msg;
}

robot_controller_ros::SourcesSupplyState AR60xRecvPacket::PowerGetSourcesSupplyState()
{
    robot_controller_ros::SourcesSupplyState msg;
    msg.S48 = sourceGetSupplyState(PowerData::Supply48V);
    msg.S12 = sourceGetSupplyState(PowerData::Supply12V);
    msg.S8_1 = sourceGetSupplyState(PowerData::Supply8V1);
    msg.S8_2 = sourceGetSupplyState(PowerData::Supply8V2);
    msg.S6_1 = sourceGetSupplyState(PowerData::Supply6V1);
    msg.S6_2 = sourceGetSupplyState(PowerData::Supply6V2);
    return msg;
}

sensor_msgs::Imu AR60xRecvPacket::SensorsGetImu()
{
    sensor_msgs::Imu imu;

    uint8_t channel = desc_.sensorGroups[ImuGroupId].channel;

    imu.linear_acceleration.x = read_int16(channel * 16 + SensorAccXOffset) / 100.0;
    imu.linear_acceleration.y = read_int16(channel * 16 + SensorAccYOffset) / 100.0;
    imu.linear_acceleration.z = read_int16(channel * 16 + SensorAccZOffset) / 100.0;
    imu.orientation.z = read_int16(channel * 16 + SensorYawOffset) / 100.0;
    imu.orientation.y = read_int16(channel * 16 + SensorPitchOffset) / 100.0;
    imu.orientation.x = read_int16(channel * 16 + SensorRollOffset) / 100.0;

    return  imu;
}


robot_controller_ros::FeetSensors AR60xRecvPacket::SensorsGetFeet()
{
    //TODO: Read feet
    throw std::runtime_error("Not implemented");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double AR60xRecvPacket::jointGetLowerLimit(JointData& joint)
{
    int16_t value;
    if(joint.isReverse)
        value = -read_int16(joint.channel * 16 + JointUpperLimitAddress);
    else
        value = read_int16(joint.channel * 16 + JointLowerLimitAddress);

    return  uint16_to_angle(value);
}

double AR60xRecvPacket::jointGetUpperLimit(JointData& joint)
{
    int16_t value;

    if(joint.isReverse)
        value = -read_int16(joint.channel * 16 + JointLowerLimitAddress);
    else
        value = read_int16(joint.channel * 16 + JointUpperLimitAddress);

    return uint16_to_angle(value);
}


robot_controller_ros::TypeJointMode AR60xRecvPacket::jointGetMode(JointData& joint)
{
    robot_controller_ros::TypeJointMode state;

    uint8_t value = byte_array_[joint.channel * 16 + JointStateAddress];
    uint8_t status_mask = value & 0b11;

    if(status_mask == 0b00)
        state.mode = JointState::BRAKE;
    else if(status_mask == 0b01)
        state.mode = JointState::STOP;
    else if(status_mask == 0b10)
        state.mode = JointState::RELAX;
    else if(status_mask == 0b11)
        state.mode = JointState::TRACE;

    // TODO: Get control type
    // TODO: Get over limits
    //state.isBeyondLowerLimit = value & (1 << 4);
    //state.isBeyondUpperLimit = value & (1 << 5);
    //state.controlType = (value & (1<<7)) ? JointState::POSITION_CONTROl : JointState::TORQUE_CONTROL;

    return state;
}


robot_controller_ros::TypeSupplyState AR60xRecvPacket::jointGetSupplyState(JointData& joint)
{
    robot_controller_ros::TypeSupplyState state;
    state.Current = read_int16(joint.channel * 16 + JointCurrentAddress) / 1000.0f;
    state.Voltage = read_int16(joint.channel * 16 + JointVoltageAddress) / 1000.0f;
    return state;
}

robot_controller_ros::TypeSupplyState AR60xRecvPacket::sourceGetSupplyState(PowerData::PowerSupplies supply)
{
    robot_controller_ros::TypeSupplyState state;

    int address = powerStateMap.at(supply).SupplyVoltageAddress;
    state.Voltage = address == Supply48VoltageAddress ? (read_float(address) / 100) : (read_float(address) / 1000);
    address = powerStateMap.at(supply).SupplyCurrentAddress;
    state.Voltage = address == Supply48VoltageAddress ? (read_float(address) / 100) : (read_float(address) / 1000);

    return state;
}

SensorFeetState::FootData AR60xRecvPacket::sensorGetFoot(uint8_t groupId)
{
    SensorFeetState::FootData data;
    uint8_t channel = desc_.sensorGroups[groupId].channel;
    data.uch0 = read_int16(channel * 16 + SensorUch0Offset) / 100.0;
    data.uch1 = read_int16(channel * 16 + SensorUch1Offset) / 100.0;
    data.uch2 = read_int16(channel * 16 + SensorUch2Offset) / 100.0;
    data.uch3 = read_int16(channel * 16 + SensorUch3Offset) / 100.0;
    //TODO: Add moments

    return data;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int16_t AR60xRecvPacket::read_int16(uint16_t address)
{
    //int16_t value =  (byte_array_[address + 1] << 8) + (BYTE)byte_array_[address];
    return  *((int16_t*)(byte_array_ + address));
}

float AR60xRecvPacket::read_float(uint16_t address)

{
    //float value = static_cast<float>((byte_array_[address + 1] << 8) + (BYTE)byte_array_[address]);
    return *((float*)(byte_array_ + address));
}

double AR60xRecvPacket::uint16_to_angle(uint16_t angle)
{
    return angle / 100.0;
}

