#include <string>
#include <termios.h>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include <mavlink/common/mavlink.h>
#pragma GCC diagnostic pop

class Port
{
private:
    int fd;
    std::string target_ip;
    int target_port;
    std::vector<uint8_t> recv_buffer;

public:
    Port(const std::string &device_path, speed_t baudrate);
    Port(const std::string &target_ip, int target_port);
    ~Port();

	bool read_message(mavlink_message_t &message);
	void write_message(const mavlink_message_t &message);
};
