// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// PrimaryPort.hpp
// Provides the PrimaryPort class for interfacing with the robot's primary port.
#ifndef __ELITE__PRIMARY_PORT_HPP__
#define __ELITE__PRIMARY_PORT_HPP__

#include "DataType.hpp"
#include "PrimaryPackage.hpp"
#include "RobotException.hpp"

#include <boost/asio.hpp>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace ELITE {

class PrimaryPort {
   private:
    // The primary port package head length
    static constexpr int HEAD_LENGTH = 5;
    // The type of 'RobotState' package
    static constexpr int ROBOT_STATE_MSG_TYPE = 16;
    // The type of 'RobotException' package
    static constexpr int ROBOT_EXCEPTION_MSG_TYPE = 20;

    std::mutex socket_mutex_;
    boost::asio::io_context io_context_;
    std::unique_ptr<boost::asio::ip::tcp::socket> socket_ptr_;

    std::function<void(RobotExceptionSharedPtr)> robot_exception_cb_;

    // The buffer of package head
    std::vector<uint8_t> message_head_;
    // The buffer of package body
    std::vector<uint8_t> message_body_;

    // When getPackage() be called, insert a sub-package data to get
    std::unordered_map<int, std::shared_ptr<PrimaryPackage>> parser_sub_msg_;
    std::unique_ptr<std::thread> socket_async_thread_;
    std::mutex mutex_;
    bool socket_async_thread_alive_;

    /**
     * @brief The background thread.
     *  Receive and parser package.
     */
    void socketAsyncLoop(const std::string& ip, int port);

    /**
     * @brief Receive and parser package.
     *
     */
    bool parserMessage();

    /**
     * @brief Receive and parser package body.
     *  Only parser 'RobotState' package
     * @param type
     * @param len
     */
    bool parserMessageBody(int type, int package_len);

    /**
     * @brief Connect to robot primary port.
     *
     * @param ip The robot ip
     * @param port The port(30001 or 30002)
     * @param is_last_connect_success Indicate whether the last connection was successful.
     * @return true
     * @return false
     */
    bool socketConnect(const std::string& ip, int port, bool is_last_connect_success = false);

    /**
     * @brief Close connection socket
     *
     */
    void socketDisconnect();

    bool socketReconnect(const std::string& ip, int port, bool is_last_connect_success);

    RobotExceptionSharedPtr parserException(const std::vector<uint8_t>& msg_body);

    RobotErrorSharedPtr parserRobotError(uint64_t timestamp, RobotError::Source source, const std::vector<uint8_t>& msg_body,
                                         int offset);

    RobotRuntimeExceptionSharedPtr paraserRuntimeException(uint64_t timestamp, const std::vector<uint8_t>& msg_body, int offset);

   public:
    PrimaryPort();
    ~PrimaryPort();

    /**
     * @brief Connect to robot primary port.
     *  And spawn a background thread for message receiving and parsing.
     * @param ip The robot ip
     * @param port The port(30001 or 30002)
     * @return true success
     * @return false fail
     * @note
     *      1. Warning: Repeated calls to this function without intermediate disconnect() will force-close the active connection.
     *      2. Usage constraint: Call rate must be ≤ 2Hz (once per 500ms minimum interval).
     */
    bool connect(const std::string& ip, int port);

    /**
     * @brief Disconnect socket.
     *  And wait for the background thread to finish.
     * @note After calling this function, a delay of around 500ms should be added prior to calling connect().
     */
    void disconnect();

    /**
     * @brief Sends a custom script program to the robot.
     *
     * @param script Script code that shall be executed by the robot.
     * @return true success
     * @return false fail
     */
    bool sendScript(const std::string& script);

    /**
     * @brief Get primary sub-package data.
     *
     * @param pkg Primary sub-package.
     * @param timeout_ms Wait time
     * @return true success
     * @return false fail
     */
    bool getPackage(std::shared_ptr<PrimaryPackage> pkg, int timeout_ms);

    /**
     * @brief Get the local IP
     *
     * @return std::string Local IP. If empty, connection had some errors.
     */
    std::string getLocalIP();

    /**
     * @brief Registers a callback for robot exceptions.
     *
     * This function registers a callback that will be invoked whenever
     * a robot exception message is received from the primary port.
     *
     * @param cb A callback function that takes a RobotExceptionSharedPtr
     *           representing the received exception.
     */
    void registerRobotExceptionCallback(std::function<void(RobotExceptionSharedPtr)> cb) { robot_exception_cb_ = cb; }
};

}  // namespace ELITE

#endif