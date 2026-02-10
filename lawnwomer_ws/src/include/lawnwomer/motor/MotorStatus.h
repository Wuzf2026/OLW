#ifndef LAWNMOWER_MOTOR_MOTORSTATUS_H
#define LAWNMOWER_MOTOR_MOTORSTATUS_H
#include <stdint.h>
#include <string>
namespace lawnmower {
struct MotorStatus {
    uint8_t id;
    std::string name;
    double speed;      // RPM
    double current;    // Amps
    double temperature; // Celsius
    uint8_t status_code;
    bool is_enabled;
    bool is_fault;
};
} // namespace lawnmower
#endif // LAWNMOWER_MOTOR_MOTORSTATUS_H
