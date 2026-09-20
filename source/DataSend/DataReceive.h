#include <iostream>
#include <vector>
#include <thread>
#include <atomic>
#include <cstring>
#include <mutex>
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")



// ========== 简单的UDP接收器 ==========
class UDPReceiver
{
public:
    UDPReceiver(int port) : running_(false), sockfd_(INVALID_SOCKET), port_(port)
    {
        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData);

        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ == INVALID_SOCKET) {
            std::cerr << "创建socket失败!" << std::endl;
            return;
        }

        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port_);
        addr.sin_addr.s_addr = INADDR_ANY;

        if (bind(sockfd_, (struct sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
            std::cerr << "绑定端口失败!" << std::endl;
            closesocket(sockfd_);
            return;
        }

        std::cout << "[OK] 监听端口: " << port_ << std::endl;
    }

    ~UDPReceiver()
    {
        stop();
        if (sockfd_ != INVALID_SOCKET) {
            closesocket(sockfd_);
            WSACleanup();
        }
    }

    void start()
    {
        if (running_) return;
        running_ = true;
        recv_thread_ = std::thread(&UDPReceiver::receiveLoop, this);
    }

    void stop()
    {
        if (!running_) return;
        running_ = false;
        if (recv_thread_.joinable()) {
            recv_thread_.join();
        }
        std::cout << "[OK] 接收停止" << std::endl;
    }

    // 设置回调函数（数据传给外面）
    void setCallback(std::function<void(const std::string&)> callback)
    {
        data_callback_ = callback;
    }

private:
    void receiveLoop()
    {
        char buffer[2048];
        struct sockaddr_in client_addr;
        int addr_len = sizeof(client_addr);

        std::cout << "[OK] 开始接收..." << std::endl;

        while (running_)
        {
            int received = recvfrom(sockfd_, buffer, sizeof(buffer) - 1, 0,
                (struct sockaddr*)&client_addr, &addr_len);

            if (received > 0) {
                buffer[received] = '\0';
                std::string data(buffer);

                // 回调给外面处理
                if (data_callback_) {
                    data_callback_(data);
                }
            }
        }
    }

private:
    std::atomic<bool> running_;
    std::thread recv_thread_;
    SOCKET sockfd_;
    int port_;
    std::function<void(const std::string&)> data_callback_;
};

