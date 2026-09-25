#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>

#include <ltr/robot/channel/channel_factory.hpp>
#include <ltr/robot/channel/channel_publisher.hpp>
#include "SportModeCmd_.hpp"
#include "VelCmd_.hpp"

namespace {
using Clock = std::chrono::steady_clock;
using ModePublisher = ltr::robot::ChannelPublisher<ltr::msg::dds_::SportModeCmd_>;
using VelocityPublisher = ltr::robot::ChannelPublisher<ltr::msg::dds_::VelCmd_>;

constexpr auto kPeriod = std::chrono::milliseconds(100);
constexpr auto kTimeout = std::chrono::milliseconds(350);

bool send_zero(VelocityPublisher& pub) {
    ltr::msg::dds_::VelCmd_ message;
    return pub.Write(message, 0);
}

bool send_velocity(VelocityPublisher& pub, float vx, float vy, float wz) {
    ltr::msg::dds_::VelCmd_ message;
    message.lin_vx(vx);
    message.lin_vy(vy);
    message.ang_wz(wz);
    return pub.Write(message, 0);
}

void stop_repeatedly(VelocityPublisher& pub) {
    for (int i = 0; i < 5; ++i) {
        send_zero(pub);
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }
}

bool valid_velocity(float vx, float vy, float wz) {
    return std::isfinite(vx) && std::isfinite(vy) && std::isfinite(wz) &&
           std::abs(vx) <= 2.0f && std::abs(vy) <= 0.6f &&
           std::abs(wz) <= 0.8f;
}
}  // namespace

int main(int argc, char** argv) {
    try {
        std::string iface;
        if (argc == 3 && std::string(argv[1]) == "--iface") {
            iface = argv[2];
        } else if (argc != 1) {
            std::cerr << "DOG_ERROR usage: dds_bridge [--iface IFACE]\n";
            return 2;
        }

        ltr::robot::ChannelFactory::Instance()->Init(0, iface);
        ModePublisher mode_pub("rt/ltr/sportmode_cmd");
        VelocityPublisher velocity_pub("rt/ltr/vel_cmd");
        mode_pub.InitChannel();
        velocity_pub.InitChannel();
        std::cerr << "DOG_READY DDS publisher initialized" << std::endl;

        float vx = 0.0f, vy = 0.0f, wz = 0.0f;
        bool moving = false;
        auto last_drive = Clock::now();
        auto last_publish = Clock::now() - kPeriod;
        std::string pending;
        char buffer[512];

        for (;;) {
            const auto now = Clock::now();
            if (moving && now - last_drive > kTimeout) {
                moving = false;
                vx = vy = wz = 0.0f;
                send_zero(velocity_pub);
                std::cerr << "DOG_WATCHDOG velocity expired" << std::endl;
            }
            if (now - last_publish >= kPeriod) {
                const bool sent = send_velocity(velocity_pub, vx, vy, wz);
                if (!sent && moving) {
                    moving = false;
                    vx = vy = wz = 0.0f;
                    std::cerr << "DOG_ERROR velocity publish failed" << std::endl;
                }
                last_publish = now;
            }

            pollfd input{STDIN_FILENO, POLLIN, 0};
            const int result = poll(&input, 1, 25);
            if (result < 0) {
                if (errno == EINTR) continue;
                std::cerr << "DOG_ERROR stdin poll failed\n";
                break;
            }
            if (result == 0) continue;
            if (input.revents & (POLLERR | POLLHUP | POLLNVAL)) {
                // Read any final buffered command before treating EOF as shutdown.
                if (!(input.revents & POLLIN)) break;
            }
            if (!(input.revents & POLLIN)) continue;
            const ssize_t count = read(STDIN_FILENO, buffer, sizeof(buffer));
            if (count <= 0) break;
            pending.append(buffer, static_cast<size_t>(count));
            if (pending.size() > 4096) {
                pending.clear();
                std::cerr << "DOG_ERROR input too long" << std::endl;
                continue;
            }
            size_t newline;
            while ((newline = pending.find('\n')) != std::string::npos) {
                const std::string line = pending.substr(0, newline);
                pending.erase(0, newline + 1);
                std::istringstream stream(line);
                std::string command, extra;
                stream >> command;
                if (command == "QUIT") {
                    stop_repeatedly(velocity_pub);
                    return 0;
                }
                if (command == "STOP") {
                    moving = false;
                    vx = vy = wz = 0.0f;
                    stop_repeatedly(velocity_pub);
                    std::cerr << "DOG_OK stopped" << std::endl;
                } else if (command == "MODE") {
                    int mode = -1;
                    if (!(stream >> mode) || (stream >> extra) || mode < 0 || mode > 4) {
                        std::cerr << "DOG_ERROR invalid mode" << std::endl;
                        continue;
                    }
                    moving = false;
                    vx = vy = wz = 0.0f;
                    send_zero(velocity_pub);
                    ltr::msg::dds_::SportModeCmd_ message;
                    message.device_id(1);  // HIGHLEVELSDK
                    message.mode(static_cast<uint8_t>(mode));
                    bool sent = true;
                    for (int i = 0; i < 3; ++i) {
                        sent = mode_pub.Write(message, 0) && sent;
                        std::this_thread::sleep_for(std::chrono::milliseconds(40));
                    }
                    std::cerr << (sent ? "DOG_OK mode " : "DOG_ERROR mode publish failed ")
                              << mode << std::endl;
                } else if (command == "VEL") {
                    float next_vx, next_vy, next_wz;
                    if (!(stream >> next_vx >> next_vy >> next_wz) ||
                        (stream >> extra) || !valid_velocity(next_vx, next_vy, next_wz)) {
                        std::cerr << "DOG_ERROR invalid velocity" << std::endl;
                        continue;
                    }
                    vx = next_vx;
                    vy = next_vy;
                    wz = next_wz;
                    moving = vx != 0.0f || vy != 0.0f || wz != 0.0f;
                    last_drive = Clock::now();
                    if (!send_velocity(velocity_pub, vx, vy, wz)) {
                        moving = false;
                        vx = vy = wz = 0.0f;
                        std::cerr << "DOG_ERROR velocity publish failed" << std::endl;
                    }
                    last_publish = Clock::now();
                } else {
                    std::cerr << "DOG_ERROR unknown command" << std::endl;
                }
            }
        }
        stop_repeatedly(velocity_pub);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "DOG_ERROR " << error.what() << std::endl;
        return 1;
    }
}
