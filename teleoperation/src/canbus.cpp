#include "canbus.h"
#include <poll.h>

CANBus::CANBus(const std::string& interface) {
    struct ifreq ifr;
    struct sockaddr_can addr;

    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
        perror("Error while opening CAN socket");
        exit(EXIT_FAILURE);
    }

    std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ);
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
        perror("Error getting interface index");
        exit(EXIT_FAILURE);
    }

    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("Error in CAN socket bind");
        exit(EXIT_FAILURE);
    }
}

CANBus::~CANBus() {
    close(sock_);
}

bool CANBus::send(uint16_t motor_id, const std::array<uint8_t, 8>& data) {
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));

    frame.can_id = motor_id;
    frame.can_dlc = data.size();
    std::copy(data.begin(), data.end(), frame.data);

    if (write(sock_, &frame, sizeof(frame)) != sizeof(frame)) {
        perror("Error sending CAN frame");
        return false;
    } else {
        // std::cout << "Sent CAN frame to motor ID " << motor_id << std::endl;
        return true;
    }
}

struct can_frame CANBus::recv() {
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));

    // // change socket into non blocking mode
    // int flags = fcntl(sock_, F_GETFL, 0);
    // fcntl(sock_, F_SETFL, flags | O_NONBLOCK);

    int nbytes = read(sock_, &frame, sizeof(struct can_frame));

    if (nbytes < 0) {
        perror("CAN read error");
    } else {
        // std::cout << "Received CAN frame from motor ID " << frame.can_id << std::endl;
    }

    return frame;
}
