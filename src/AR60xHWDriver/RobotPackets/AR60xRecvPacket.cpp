#include "AR60xRecvPacket.h"

AR60xRecvPacket::AR60xRecvPacket(AR60xDescription& robotDesc) : BasePacket(robotDesc)
{
}


void AR60xRecvPacket::initFromByteArray(const uint8_t *bytes)
{
    memcpy(byte_array_, bytes, packetSize);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


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


SensorFeetState AR60xRecvPacket::SensorsGetFeet()
{
    SensorFeetState state;
    state.left = sensorGetFoot(LeftFootGroupId);
    state.right = sensorGetFoot(RightFootGroupId);
    return state;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


double AR60xRecvPacket::JointGetPosition(JointData &joint)
{
    return int16_to_angle(read_int16(joint.channel * 16 + JointPositionAddress));
}

double AR60xRecvPacket::JointGetLowerLimit(JointData &joint)
{
    int16_t value;
    if(joint.isReverse)
        value = -read_int16(joint.channel * 16 + JointUpperLimitAddress);
    else
        value = read_int16(joint.channel * 16 + JointLowerLimitAddress);

    return int16_to_angle(value);
}

double AR60xRecvPacket::JointGetUpperLimit(JointData &joint)
{
    int16_t value;

    if(joint.isReverse)
        value = -read_int16(joint.channel * 16 + JointLowerLimitAddress);
    else
        value = read_int16(joint.channel * 16 + JointUpperLimitAddress);

    return int16_to_angle(value);
}


double AR60xRecvPacket::JointGetOffset(JointData &joint)
{
    return int16_to_angle(read_int16(joint.channel * 16 + JointOffsetAddress));
}

robot_controller_ros::TypeJointMode AR60xRecvPacket::JointGetMode(JointData &joint)
{
    robot_controller_ros::TypeJointMode state;

    uint8_t value = byte_array_[joint.channel * 16 + JointStateAddress];
    uint8_t status_mask = value & 0b11;

    if(status_mask == 0b00)
        state.mode = robot_controller_ros::TypeJointMode::BREAK;
    else if(status_mask == 0b01)
        state.mode = robot_controller_ros::TypeJointMode::STOP;
    else if(status_mask == 0b10)
        state.mode = robot_controller_ros::TypeJointMode::RELAX;
    else if(status_mask == 0b11)
        state.mode = robot_controller_ros::TypeJointMode::TRACE;

    // TODO: Get control type
    // TODO: Get over limits
    //state.isBeyondLowerLimit = value & (1 << 4);
    //state.isBeyondUpperLimit = value & (1 << 5);
    //state.controlType = (value & (1<<7)) ? JointState::POSITION_CONTROl : JointState::TORQUE_CONTROL;

    return state;
}


robot_controller_ros::TypePid AR60xRecvPacket::JointGetPidGains(JointData &joint)
{
    //FUCK Android Technology company
    robot_controller_ros::TypePid pid;
    pid.p = read_int16(joint.channel * 16 + JointPGainAddress);
    pid.i = read_int16(joint.channel * 16 + JointIGainAddress);
    pid.d = joint.gains.d; // We can't read D gain from robot
}


robot_controller_ros::TypeSupplyState AR60xRecvPacket::PowerGetJointSupplyState(JointData &joint)
{
    robot_controller_ros::TypeSupplyState state;
    state.Current = read_int16(joint.channel * 16 + JointCurrentAddress) / 1000.0f;
    state.Voltage = read_int16(joint.channel * 16 + JointVoltageAddress) / 1000.0f;
    return state;
}

robot_controller_ros::TypeSupplyState AR60xRecvPacket::PowerGetSourceSupplyState(PowerSources supply)
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

double AR60xRecvPacket::int16_to_angle(int16_t angle)
{
    return angle / 100.0;
}



