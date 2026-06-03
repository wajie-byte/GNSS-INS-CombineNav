// simple_udp.h
#pragma once

#include <string>
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

class SimpleUdp {
public:
    SimpleUdp(const std::string& ip = "127.0.0.1", int port = 5005) {
        WSADATA wsa;
        WSAStartup(MAKEWORD(2, 2), &wsa);
        sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

        memset(&dest_, 0, sizeof(dest_));
        dest_.sin_family = AF_INET;
        dest_.sin_port = htons(port);
        inet_pton(AF_INET, ip.c_str(), &dest_.sin_addr);
    }

    ~SimpleUdp() {
        if (sock_ != INVALID_SOCKET) closesocket(sock_);
        WSACleanup();
    }

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

private:
    SOCKET sock_;
    sockaddr_in dest_;
};