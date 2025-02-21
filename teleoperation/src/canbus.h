#ifndef CANBUS_H
#define CANBUS_H

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <array>
#include <fcntl.h>

class CANBus {
public:
    explicit CANBus(const std::string& interface);
    ~CANBus();
    
    bool send(uint16_t motor_id, const std::array<uint8_t, 8>& data);
    struct can_frame recv();

private:
    int sock_;
};

#endif // CANBUS_H
