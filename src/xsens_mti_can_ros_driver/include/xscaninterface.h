#ifndef XSCANINTERFACE_H
#define XSCANINTERFACE_H

#include <atomic>
#include <chrono>
#include <list>
#include <memory>
#include <string>
#include <thread>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include "xsens_parser.h"
#include "xsens_time_handler.h"

class PacketCallback;

class XsCanInterface
{
public:
    explicit XsCanInterface(rclcpp::Node::SharedPtr node);
    ~XsCanInterface();

    // Declare and read parameters, open the SocketCAN socket. Returns true on
    // success; false leaves the driver inert (no read thread, no callbacks).
    bool initialize();
    // Construct publishers based on pub_* parameters.
    void registerPublishers();
    // Spawn the dedicated CAN read thread. No-op if already running or if
    // initialize() failed.
    void start();
    // Signal the read thread to stop and join it; close the socket. Idempotent.
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

    // Diagnostics — published at ~1 Hz via diagnostic_updater. Counters are
    // updated from the reader thread; updater publish runs on a wall-clock
    // timer (single thread, but cheap to read atomics).
    std::unique_ptr<diagnostic_updater::Updater> diag_updater_;
    std::atomic<uint64_t> good_frames_{0};
    std::atomic<uint64_t> bad_frames_{0};
    std::atomic<uint64_t> can_err_frames_{0};
    std::atomic<uint64_t> unknown_frames_{0};
    std::atomic<uint64_t> dropped_samples_{0};
    std::atomic<uint64_t> read_errors_{0};
    std::atomic<bool> socket_healthy_{false};
    std::atomic<bool> error_latched_{false};   // Set on XsError frame
    std::atomic<bool> warning_latched_{false}; // Set on XsWarning frame

    // Group-counter integrity: if the wire counter doesn't increment by 1
    // between two start-frame dispatches, drop the sample (we lost frames).
    bool prev_counter_valid_;
    uint16_t prev_counter_value_;

    // Throttling for read-error logs.
    rclcpp::Time last_read_error_log_;

    // Configuration loaded from parameters.
    int read_timeout_ms_;
    int reopen_after_errors_;

    bool openSocket();
    void closeSocket();
    void registerCallback(std::unique_ptr<PacketCallback> cb);
    void readLoop();
    void processCANMessages();
    void saveToPacket(const can_frame &frame, XsDataPacket &packet);
    void dispatchPacket();
    void diagInterface(diagnostic_updater::DiagnosticStatusWrapper &stat);
};

#endif // XSCANINTERFACE_H
