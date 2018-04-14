#include "AR60xRecvPacket.h"

AR60xRecvPacket::AR60xRecvPacket(AR60xDescription& robotDesc) : BasePacket(robotDesc)
{
}


void AR60xRecvPacket::initFromByteArray(const uint8_t *bytes)
{
    memcpy(byte_array_, bytes, packetSize);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


double AR60xRecvPacket::JointGetPosition(const JointData &joint) const
{
    return  deg2rad(int16_to_angle(read_int16(joint.channel * 16 + JointPositionAddress)));
}

double AR60xRecvPacket::JointGetLowerLimit(const JointData &joint) const
{
    //TODO: deg2rad

    int16_t value;
    if(joint.is_reverse)
        value = -read_int16(joint.channel * 16 + JointUpperLimitAddress);
    else
        value = read_int16(joint.channel * 16 + JointLowerLimitAddress);

    return int16_to_angle(value);
}

double AR60xRecvPacket::JointGetUpperLimit(const JointData &joint) const
{
    //TODO: deg2rad

    int16_t value;

    if(joint.is_reverse)
        value = -read_int16(joint.channel * 16 + JointLowerLimitAddress);
    else
        value = read_int16(joint.channel * 16 + JointUpperLimitAddress);

    return int16_to_angle(value);
}


double AR60xRecvPacket::JointGetOffset(const JointData &joint) const
{
    //TODO: deg2rad
    return int16_to_angle(read_int16(joint.channel * 16 + JointOffsetAddress));
}

robot_controller_ros::TypeJointMode AR60xRecvPacket::JointGetMode(const JointData &joint) const
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


robot_controller_ros::TypePid AR60xRecvPacket::JointGetPidGains(const JointData &joint) const
{
    //FUCK Android Technology company
    robot_controller_ros::TypePid pid;
    pid.p = read_int16(joint.channel * 16 + JointPGainAddress);
    pid.i = read_int16(joint.channel * 16 + JointIGainAddress);
    pid.d = joint.pid_gains.d; // We can't read D gain from robot

    return pid;
}


robot_controller_ros::TypeSupplyState AR60xRecvPacket::PowerGetJointSupplyState(const JointData &joint) const
{
    robot_controller_ros::TypeSupplyState state;
    state.Current = read_int16(joint.channel * 16 + JointCurrentAddress) / 100.0f;
    state.Voltage = read_int16(joint.channel * 16 + JointVoltageAddress) / 100.0f;
    return state;
}

robot_controller_ros::TypeSupplyState AR60xRecvPacket::PowerGetSourceSupplyState(const PowerSources supply) const
{
    robot_controller_ros::TypeSupplyState state;

    state.Voltage = read_float(powerStateMap.at(supply).SupplyVoltageAddress) / 1000;
    state.Current = read_float(powerStateMap.at(supply).SupplyCurrentAddress) / 1000;

    if(supply == PowerSources::Supply48V)
    {
        state.Voltage *=10;
        state.Current *= 10;
    }

    return state;
}

sensor_msgs::Imu AR60xRecvPacket::SensorsGetImu() const
{
    sensor_msgs::Imu imu;

    uint8_t channel = desc_.sensorGroups[ImuGroupId].channel;

    imu.linear_acceleration.x = read_int16(channel * 16 + SensorAccXOffset) / 100.0;
    imu.linear_acceleration.y = read_int16(channel * 16 + SensorAccYOffset) / 100.0;
    imu.linear_acceleration.z = read_int16(channel * 16 + SensorAccZOffset) / 100.0;
    imu.orientation.z = read_int16(channel * 16 + SensorYawOffset) / 100.0;
    imu.orientation.y = read_int16(channel * 16 + SensorPitchOffset) / 100.0;
    imu.orientation.x = read_int16(channel * 16 + SensorRollOffset) / 100.0;
    imu.header.frame_id = "imu";  //TODO: Maybe should read from config?

    return  imu;
}


robot_controller_ros::FeetSensors AR60xRecvPacket::SensorsGetFeet() const
{
    robot_controller_ros::FeetSensors state;
    state.left = sensorGetFoot(LeftFootGroupId);
    state.left = sensorGetFoot(RightFootGroupId);
    return state;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

robot_controller_ros::TypeFootSensor AR60xRecvPacket::sensorGetFoot(const uint8_t groupId) const
{
    robot_controller_ros::TypeFootSensor data;
    uint8_t channel = desc_.sensorGroups[groupId].channel;

    data.uch0 = read_int16(channel * 16 + SensorUch0Offset) / 100.0f;
    data.uch1 = read_int16(channel * 16 + SensorUch1Offset) / 100.0f;
    data.uch2 = read_int16(channel * 16 + SensorUch2Offset) / 100.0f;
    data.uch3 = read_int16(channel * 16 + SensorUch3Offset) / 100.0f;

    data.tx = read_int16(channel * 16 + SensorTyOffset) / 100.0f;
    data.ty = read_int16(channel * 16 + SensorTyOffset) / 100.0f;
    data.fz = read_int16(channel * 16 + SensorFzOffset) / 100.0f;

    return data;
}

int16_t AR60xRecvPacket::read_int16(const uint16_t address) const
{
    // FUCK AT: Big Endian
    return  (byte_array_[address + 1] << 8) + (BYTE)byte_array_[address];
}

float AR60xRecvPacket::read_float(const uint16_t address) const
{
    // FUCK AT: Big Endian
    return static_cast<float>((byte_array_[address + 1] << 8) + (BYTE)byte_array_[address]);
}

double AR60xRecvPacket::int16_to_angle(const int16_t angle) const
{
    return angle / 100.0;
}

double AR60xRecvPacket::deg2rad(const double deg) const
{
    return deg / 180.0 * M_PI;
}

