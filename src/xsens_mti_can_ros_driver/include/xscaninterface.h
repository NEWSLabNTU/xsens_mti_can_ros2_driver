#ifndef XSCANINTERFACE_H
#define XSCANINTERFACE_H

#include <atomic>
#include <list>
#include <memory>
#include <string>
#include <thread>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <rclcpp/rclcpp.hpp>

#include "xsens_parser.h"
#include "xsens_time_handler.h"

class PacketCallback;

class XsCanInterface
{
public:
    explicit XsCanInterface(rclcpp::Node::SharedPtr node);
    ~XsCanInterface();

    // Declare and read parameters, open the SocketCAN socket.
    void initialize();
    // Construct publishers based on pub_* parameters.
    void registerPublishers();
    // Spawn the dedicated CAN read thread.
    void start();
    // Signal the read thread to stop and join it; close the socket.
    void stop();

private:
    rclcpp::Node::SharedPtr node_;
    int socket_;
    std::string can_interface_;
    XsDataPacket packet_;
    uint32_t m_startframeid;
    uint32_t lastFrameId_;
    std::list<std::unique_ptr<PacketCallback>> m_callbacks;
    XsensTimeHandler m_timeHandler;

    std::thread reader_thread_;
    std::atomic<bool> running_;

    void registerCallback(std::unique_ptr<PacketCallback> cb);
    void readLoop();
    void processCANMessages();
    void saveToPacket(const can_frame &frame, XsDataPacket &packet);
};

#endif // XSCANINTERFACE_H
