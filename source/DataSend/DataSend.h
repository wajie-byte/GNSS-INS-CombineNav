// simple_udp.h
#pragma once

#include <string>
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

class SimpleUdp {
public:
    // 发送端构造函数（指定目标IP和端口）
    SimpleUdp(const std::string& ip, int port) {
        WSADATA wsa;
        WSAStartup(MAKEWORD(2, 2), &wsa);
        sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

        memset(&dest_, 0, sizeof(dest_));
        dest_.sin_family = AF_INET;
        dest_.sin_port = htons(port);
        inet_pton(AF_INET, ip.c_str(), &dest_.sin_addr);

        is_receiver_ = false;
    }

    // 接收端构造函数（绑定本地端口）
    SimpleUdp(int local_port) {
        WSADATA wsa;
        WSAStartup(MAKEWORD(2, 2), &wsa);
        sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

        sockaddr_in local_addr;
        memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sin_family = AF_INET;
        local_addr.sin_port = htons(local_port);
        local_addr.sin_addr.s_addr = INADDR_ANY;  // 接收任意IP发来的数据

        bind(sock_, (sockaddr*)&local_addr, sizeof(local_addr));

        is_receiver_ = true;
    }

    ~SimpleUdp() {
        if (sock_ != INVALID_SOCKET) closesocket(sock_);
        WSACleanup();
    }

    // 发送数据（原功能保持不变）
    void send(double t,
        double lat, double lon, double alt,
        double vn, double ve, double vd,
        double roll, double pitch, double yaw) {
        char buf[512];
        sprintf_s(buf,
            "{\"t\":%.6f,\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.3f,"
            "\"vn\":%.3f,\"ve\":%.3f,\"vd\":%.3f,"
            "\"roll\":%.4f,\"pitch\":%.4f,\"yaw\":%.4f}",
            t, lat, lon, alt, vn, ve, vd, roll, pitch, yaw);

        sendto(sock_, buf, (int)strlen(buf), 0,
            (sockaddr*)&dest_, sizeof(dest_));
    }

    // 接收数据（非阻塞，返回接收到的字节数，0表示无数据，-1表示错误）
    int recv(char* buf, int buf_size) {
        if (!is_receiver_) return -1;

        fd_set read_set;
        FD_ZERO(&read_set);
        FD_SET(sock_, &read_set);

        timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 1000;  // 1ms超时

        int select_result = select(0, &read_set, nullptr, nullptr, &timeout);

        if (select_result <= 0)
            return 0;  // 超时或错误，无数据

        sockaddr_in sender_addr;
        int sender_addr_size = sizeof(sender_addr);

        return recvfrom(sock_, buf, buf_size, 0,
            (sockaddr*)&sender_addr, &sender_addr_size);
    }

private:
    SOCKET sock_;
    sockaddr_in dest_;
    bool is_receiver_;
};