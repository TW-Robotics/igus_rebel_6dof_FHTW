#include "igus_rebel/RebelSocket.hpp"
#include "igus_rebel/CriKeywords.hpp"

#include "rclcpp/rclcpp.hpp"

#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <sys/time.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

namespace Igus
{
    //
    // Constructor(s)/ Destructor(s)
    //
    RebelSocket::RebelSocket(const std::string &ip, const int &port, const int &timeout)
        : ip(ip),
          port(port),
          timeout(timeout),
          unprocessedMessages()
    {
    }

    RebelSocket::~RebelSocket()
    {
        Stop();
    }

    //
    // private functions
    //

    // Makes one connection attempt. Must not be called while a connection is open.
    bool RebelSocket::OpenConnection()
    {
        int fd = socket(AF_INET, SOCK_STREAM, 0);

        if (fd < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("igus_rebel"), "Socket creation error: %s", strerror(errno));
            return false;
        }

        // On Linux the send timeout also bounds connect(), so an unreachable robot cannot block us for long.
        struct timeval sendTimeout;
        sendTimeout.tv_sec = sendTimeoutMs / 1000;
        sendTimeout.tv_usec = (sendTimeoutMs % 1000) * 1000;
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &sendTimeout, sizeof(sendTimeout));

        // The receive timeout lets the receive thread check regularly whether it should stop.
        struct timeval receiveTimeout;
        receiveTimeout.tv_sec = timeout / 1000;
        receiveTimeout.tv_usec = (timeout % 1000) * 1000;
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &receiveTimeout, sizeof(receiveTimeout));

        // ALIVEJOG messages are small and time critical, do not let the kernel batch them.
        int noDelay = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &noDelay, sizeof(noDelay));

        struct sockaddr_in serv_addr;
        std::memset(&serv_addr, 0, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port);

        if (inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr) <= 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("igus_rebel"), "Invalid robot IP address / Address not supported: %s", ip.c_str());
            close(fd);
            return false;
        }

        if (connect(fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
        {
            // Only log the first failure of a series, retries happen every few hundred ms.
            if (!connectFailureLogged)
            {
                RCLCPP_ERROR(rclcpp::get_logger("igus_rebel"), "Connection to ReBeL at %s:%d failed: %s. Retrying...",
                             ip.c_str(), port, strerror(errno));
                connectFailureLogged = true;
            }
            close(fd);
            return false;
        }

        connectFailureLogged = false;
        receiveBuffer.clear();

        {
            std::lock_guard<std::mutex> lockGuard(socketWriteLock);
            sock = fd;
            connected = true;
        }
        connectionCount++;

        RCLCPP_INFO(rclcpp::get_logger("igus_rebel"), "Connected to ReBeL at %s:%d", ip.c_str(), port);
        return true;
    }

    void RebelSocket::CloseConnection()
    {
        std::lock_guard<std::mutex> lockGuard(socketWriteLock);

        if (sock >= 0)
        {
            shutdown(sock, SHUT_RDWR);
            close(sock);
            sock = -1;
        }

        connected = false;
    }

    void RebelSocket::PushMessage(std::string &&msg)
    {
        {
            std::lock_guard<std::mutex> lockGuard(messageLock);
            unprocessedMessages.push_front(std::move(msg));

            // Make sure that we do not fill our entire memory with messages from the robot in case something
            // goes wrong with processing them.
            if (unprocessedMessages.size() > maxUnprocessedMessages)
            {
                unprocessedMessages.pop_back();

                auto now = std::chrono::steady_clock::now();
                if (now - lastDiscardWarning > std::chrono::seconds(5))
                {
                    RCLCPP_WARN(rclcpp::get_logger("igus_rebel"), "Robot messages are not processed fast enough. Discarding messages.");
                    lastDiscardWarning = now;
                }
            }
        }

        messageCondition.notify_one();
    }

    // Splits receiveBuffer into complete "CRISTART ... CRIEND" messages. Incomplete data stays
    // in the buffer until the rest arrives with the next read.
    void RebelSocket::SeparateMessages()
    {
        const std::string &START = CriKeywords::START;
        const std::string &END = CriKeywords::END;

        while (true)
        {
            std::string::size_type start = receiveBuffer.find(START);

            if (start == std::string::npos)
            {
                // Keep the tail, it might be the beginning of a START keyword.
                if (receiveBuffer.size() > START.size())
                {
                    receiveBuffer.erase(0, receiveBuffer.size() - START.size());
                }
                return;
            }

            std::string::size_type end = receiveBuffer.find(END, start + START.size());

            if (end == std::string::npos)
            {
                // Message not complete yet.
                receiveBuffer.erase(0, start);

                if (receiveBuffer.size() > bufferSize * 4)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("igus_rebel"), "Received a message without end from the robot. Discarding it.");
                    receiveBuffer.clear();
                }
                return;
            }

            // A START before the END means the previous message was cut off, skip it.
            std::string::size_type nextStart = receiveBuffer.find(START, start + START.size());

            if (nextStart != std::string::npos && nextStart < end)
            {
                receiveBuffer.erase(0, nextStart);
                continue;
            }

            // Message content without "CRISTART " and " CRIEND"
            std::string::size_type contentStart = start + START.size() + 1;

            if (end > contentStart + 1)
            {
                PushMessage(receiveBuffer.substr(contentStart, end - contentStart - 1));
            }

            receiveBuffer.erase(0, end + END.size());
        }
    }

    void RebelSocket::ReceiveThreadFunction()
    {
        RCLCPP_DEBUG(rclcpp::get_logger("igus_rebel"), "Starting to receive messages from robot.");

        char buffer[bufferSize];

        while (continueReceive)
        {
            if (!connected)
            {
                // Clean up whatever is left of the old connection before making a new one.
                CloseConnection();

                if (!OpenConnection())
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(reconnectWaitMs));
                    continue;
                }
            }

            ssize_t valread = recv(sock, buffer, bufferSize, 0);

            if (valread > 0)
            {
                receiveBuffer.append(buffer, valread);
                SeparateMessages();
            }
            else if (valread == 0)
            {
                RCLCPP_WARN(rclcpp::get_logger("igus_rebel"), "Robot closed the connection.");
                CloseConnection();
            }
            else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
            {
                RCLCPP_ERROR(rclcpp::get_logger("igus_rebel"), "Socket error: %s", strerror(errno));
                CloseConnection();
            }
        }

        RCLCPP_DEBUG(rclcpp::get_logger("igus_rebel"), "Stopped to receive messages from robot.");
    }

    //
    // public functions
    //
    bool RebelSocket::Start(const int &connectTimeoutMs)
    {
        Stop();

        connectFailureLogged = false;
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(connectTimeoutMs);

        while (!OpenConnection())
        {
            if (std::chrono::steady_clock::now() >= deadline)
            {
                return false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(reconnectWaitMs));
        }

        continueReceive = true;
        receiveThread = std::thread(&RebelSocket::ReceiveThreadFunction, this);

        return true;
    }

    void RebelSocket::Stop()
    {
        continueReceive = false;

        if (receiveThread.joinable())
        {
            receiveThread.join();
        }

        CloseConnection();

        std::lock_guard<std::mutex> lockGuard(messageLock);
        unprocessedMessages.clear();
    }

    bool RebelSocket::IsConnected() const
    {
        return connected;
    }

    unsigned int RebelSocket::ConnectionCount() const
    {
        return connectionCount;
    }

    bool RebelSocket::WaitForMessage(std::string &msg, const int &timeoutMs)
    {
        std::unique_lock<std::mutex> lock(messageLock);

        if (!messageCondition.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                                       [this]
                                       { return !unprocessedMessages.empty(); }))
        {
            return false;
        }

        msg = std::move(unprocessedMessages.back());
        unprocessedMessages.pop_back();

        return true;
    }

    bool RebelSocket::SendMessage(const std::string &msg)
    {
        std::lock_guard<std::mutex> lockGuard(socketWriteLock);

        if (!connected || sock < 0)
        {
            return false;
        }

        std::string::size_type totalSent = 0;

        while (totalSent < msg.length())
        {
            // MSG_NOSIGNAL: a broken connection must not kill the whole process with SIGPIPE.
            ssize_t sent = send(sock, msg.c_str() + totalSent, msg.length() - totalSent, MSG_NOSIGNAL);

            if (sent < 0)
            {
                if (errno == EINTR)
                {
                    continue;
                }

                RCLCPP_ERROR(rclcpp::get_logger("igus_rebel"), "Sending to robot failed: %s", strerror(errno));

                // Wake up the receive thread, it closes the socket and reconnects.
                shutdown(sock, SHUT_RDWR);
                connected = false;
                return false;
            }

            totalSent += sent;
        }

        return true;
    }
}
