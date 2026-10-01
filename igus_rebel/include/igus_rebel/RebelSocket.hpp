#ifndef REBEL_SOCKET_HPP_
#define REBEL_SOCKET_HPP_
#include "rclcpp/rclcpp.hpp"
#include <stdio.h>
#include <sys/socket.h>
#include <string>
#include <list>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <thread>
#include <mutex>

namespace Igus
{
    class RebelSocket
    {
    private:
        std::atomic<int> sock{-1};
        std::string ip;
        int port;
        int timeout; // Receive timeout in ms, so the receive thread can notice a stop request
        std::list<std::string> unprocessedMessages;

        std::atomic<bool> continueReceive{false};
        std::atomic<bool> connected{false};
        std::atomic<unsigned int> connectionCount{0};
        std::thread receiveThread;
        std::mutex socketWriteLock;
        std::mutex messageLock;
        std::condition_variable messageCondition;
        unsigned long maxUnprocessedMessages = 200;
        std::chrono::steady_clock::time_point lastDiscardWarning;

        static constexpr int bufferSize = 4096;
        static constexpr int sendTimeoutMs = 1000;
        static constexpr int reconnectWaitMs = 500;

        // Bytes received but not yet split into complete messages (only used by the receive thread)
        std::string receiveBuffer;
        bool connectFailureLogged = false;

        bool OpenConnection();
        void CloseConnection();
        void SeparateMessages();
        void PushMessage(std::string &&);

        void ReceiveThreadFunction();

    public:
        RebelSocket(const std::string &, const int &, const int &);
        ~RebelSocket();

        // Connects to the robot (retrying for at most connectTimeoutMs) and starts receiving.
        // Returns false if no connection could be made in time.
        bool Start(const int &connectTimeoutMs);
        void Stop();
        bool IsConnected() const;
        // Incremented on every successful (re)connect, so users can detect reconnects.
        unsigned int ConnectionCount() const;
        // Waits up to timeoutMs for a message. Returns false if none arrived.
        bool WaitForMessage(std::string &, const int &timeoutMs);
        bool SendMessage(const std::string &);
    };
}

#endif
