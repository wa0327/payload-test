#include "port.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

Port::Port(const std::string &device_path, speed_t baudrate)
{
    fd = open(device_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
    {
        perror("open");
        throw EXIT_FAILURE;
    }

    struct termios tty{};
    tcgetattr(fd, &tty);
    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag = tty.c_oflag = tty.c_lflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
    {
        perror("tcsetattr");
        close(fd);
        throw EXIT_FAILURE;
    }
}

Port::Port(const std::string &target_ip, int target_port)
    : target_ip{target_ip}, target_port{target_port}
{
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        perror("socket");
        throw EXIT_FAILURE;
    }

    int reuse = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse, sizeof(reuse));

    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1)
    {
        perror("fcntl F_GETFL");
        close(fd);
        throw EXIT_FAILURE;
    }
    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) == -1)
    {
        perror("fcntl F_SETFL O_NONBLOCK");
        close(fd);
        throw EXIT_FAILURE;
    }
}

Port::~Port()
{
    close(fd);
}

bool Port::read_message(mavlink_message_t &message)
{
    uint8_t buf[2048];
    ssize_t len = recv(fd, buf, sizeof(buf), 0);

    if (len > 0)
    {
        recv_buffer.insert(recv_buffer.end(), buf, buf + len);
    }

    mavlink_status_t status;
    size_t i = 0;
    for (; i < recv_buffer.size(); i++)
    {
        if (mavlink_parse_char(MAVLINK_COMM_0, recv_buffer[i], &message, &status))
        {
            recv_buffer.erase(recv_buffer.begin(), recv_buffer.begin() + i + 1);
            return true;
        }
    }

    recv_buffer.erase(recv_buffer.begin(), recv_buffer.begin() + i);
    return false;
}

void Port::write_message(const mavlink_message_t &message)
{
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    size_t length = mavlink_msg_to_send_buffer(buffer, &message);

    if (target_ip.empty())
    {
        write(fd, buffer, length);
    }
    else
    {
        struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        inet_aton(target_ip.c_str(), &addr.sin_addr);
        addr.sin_port = htons(target_port);
        sendto(fd, buffer, length, 0, (struct sockaddr *)&addr, sizeof(addr));
    }
}
