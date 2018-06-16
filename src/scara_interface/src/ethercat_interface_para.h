/*
    double torqueForEmergencyStop;
    double overloadLevel;
    double overSpeedLevel;
    double motorWorkingRange;
    int interpolationTimePeriod;

    uint16 target_torque;
    uint32 max_motor_speed;
    uint16 max_torque;
    uint16 controlword;
    uint8  operation_mode;
    uint32_t profileVel;
*/
static const int NSEC_PER_SECOND = 1e+9;
static const int USEC_PER_SECOND = 1e+6;

void timespecInc(struct timespec &tick, int nsec)
{
    tick.tv_nsec += nsec;
    while (tick.tv_nsec >= NSEC_PER_SECOND)
    {
        tick.tv_nsec -= NSEC_PER_SECOND;
        tick.tv_sec++;
    }
}

