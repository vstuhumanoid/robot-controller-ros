#include "AR60xRecvPacket.h"

AR60xRecvPacket::AR60xRecvPacket(AR60xDescription& robotDesc) : BasePacket(robotDesc)
{
}


void AR60xRecvPacket::initFromByteArray(const uint8_t *bytes)
{
    memcpy(byte_array_, bytes, packetSize);
}


PowerState::PowerSupplyState AR60xRecvPacket::jointGetSupplyState(uint8_t number)
{
    PowerState::PowerSupplyState state;
    int channel = desc_.joints[number].channel;
    state.Current = read_int16(channel * 16 + JointCurrentAddress) / 1000.0f;
    state.Voltage = read_int16(channel * 16 + JointVoltageAddress) / 1000.0f;
    return state;
}


JointData::PIDGains AR60xRecvPacket::jointGetPIDGains(uint8_t number)
{
    // Can read from robot only P and I gains, but D can be read only
    // from desc.
    // Fuck AT, just return from description
    return desc_.joints[number].gains;
}

JointState AR60xRecvPacket::jointGetState(uint8_t number)
{
    JointState state;

    int channel = desc_.joints.at(number).channel;
    uint8_t value = byte_array_[channel * 16 + JointStateAddress];
    uint8_t status_mask = value & 0b11;

    if(status_mask == 0b00)
        state.state = JointState::BRAKE;
    else if(status_mask == 0b01)
        state.state = JointState::STOP;
    else if(status_mask == 0b10)
        state.state = JointState::RELAX;
    else if(status_mask == 0b11)
        state.state = JointState::TRACE;

    state.isBeyondLowerLimit = value & (1 << 4);
    state.isBeyondUpperLimit = value & (1 << 5);
    state.controlType = (value & (1<<7)) ? JointState::POSITION_CONTROl : JointState::TORQUE_CONTROL;

    return state;
}

double AR60xRecvPacket::jointGetLowerLimit(uint8_t number)
{
    int16_t value;
    
    int channel = desc_.joints.at(number).channel;
    if(desc_.joints.at(number).isReverse)
        value = -read_int16(channel * 16 + JointUpperLimitAddress);
    else
        value = read_int16(channel * 16 + JointLowerLimitAddress);
    
    return  uint16_to_angle(value);
}

double AR60xRecvPacket::jointGetUpperLimit(uint8_t number)
{
    int16_t value;
    
    int channel = desc_.joints.at(number).channel;
    if(desc_.joints.at(number).isReverse)
        value = -read_int16(channel * 16 + JointLowerLimitAddress);
    else
        value = read_int16(channel * 16 + JointUpperLimitAddress);
    
    return uint16_to_angle(value);
}

PowerState::PowerSupplyState AR60xRecvPacket::supplyGetState(PowerData::PowerSupplies supply)
{
    PowerState::PowerSupplyState state;
    state.Voltage = supplyGetVoltage(supply);
    state.Current = supplyGetCurrent(supply);
    return state;
}

float AR60xRecvPacket::supplyGetVoltage(PowerData::PowerSupplies supply)
{
   
   int address = powerStateMap.at(supply).SupplyVoltageAddress;
   float value = read_float(address) / 1000;
   if(address == Supply48VoltageAddress) value *= 10;
   
   return value;
}

float AR60xRecvPacket::supplyGetCurrent(PowerData::PowerSupplies supply)
{
    
    int address = powerStateMap.at(supply).SupplyCurrentAddress;
    float value = read_float(address) / 1000;
    if(address == Supply48CurrentAddress) value *= 10;
    
    return value;
}

double AR60xRecvPacket::sensorGetValue(short number)
{
    //TODO: Invalid. Number - sensor, not group
    throw std::runtime_error("Not implemented");
    int channel = desc_.sensorGroups[number].channel;
    int16_t value = read_int16(channel * 16 + sensorsMap.at(number));
    return value / 100;
}

SensorImuState AR60xRecvPacket::sensorGetImu()
{
    SensorImuState data;
    uint8_t channel = desc_.sensorGroups[ImuGroupId].channel;
    data.accX = read_int16(channel * 16 + SensorAccXOffset) / 100.0;
    data.accY = read_int16(channel * 16 + SensorAccYOffset) / 100.0;
    data.accZ = read_int16(channel * 16 + SensorAccZOffset) / 100.0;
    data.yaw = read_int16(channel * 16 + SensorYawOffset) / 100.0;
    data.pitch = read_int16(channel * 16 + SensorPitchOffset) / 100.0;
    data.roll = read_int16(channel * 16 + SensorRollOffset) / 100.0;

    return  data;
}

SensorFeetState AR60xRecvPacket::sensorGetFeet()
{
    SensorFeetState data;
    data.left = sensorGetFoot(LeftFootGroupId);
    data.right = sensorGetFoot(RightFootGroupId);
    return data;
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

