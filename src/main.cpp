#include "CLI11.hpp"
#include "port.hpp"
#include <iostream>
#include <csignal>

using namespace std;

bool running = true;

void quit_handler(int)
{
    running = false;
}

void send_heartbeat(Port *port)
{
    static time_t last_time = 0;
    int current_time = time(nullptr);
    if (current_time - last_time < 1)
        return;
    last_time = current_time;
    mavlink_message_t heartbeat_msg;
    mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_ONBOARD_COMPUTER, &heartbeat_msg, MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
    port->write_message(heartbeat_msg);
    cout << "Sent heartbeat" << endl;
}

void get_Camera_settings(Port *port)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        1,
        MAV_COMP_ID_ONBOARD_COMPUTER,
        &msg,
        1,
        MAV_COMP_ID_CAMERA,
        // MAV_CMD_REQUEST_MESSAGE,
        MAV_CMD_REQUEST_CAMERA_SETTINGS,
        0,
        // MAVLINK_MSG_ID_CAMERA_SETTINGS,
        1,
        0, 0, 0, 0, 0, 0);
    port->write_message(msg);

    time_t start = time(nullptr);
    while (running)
    {
        time_t now = time(nullptr);
        if (now - start > 3)
        {
            cerr << "Msg CAMERA_SETTINGS failed" << endl;
            break;
        }

        if (port->read_message(msg))
        {
            if (msg.msgid == MAVLINK_MSG_ID_CAMERA_SETTINGS)
            {
                cout << "GotMsg CAMERA_SETTINGS: ";
                mavlink_camera_settings_t t;
                mavlink_msg_camera_settings_decode(&msg, &t);
                cout << "camera_device_id=" << (int)t.camera_device_id << " zoomLevel=" << t.zoomLevel << endl;
                break;
            }
        }
    }
}

void set_camera_zoom_range(Port *port, int range)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        1,
        MAV_COMP_ID_ONBOARD_COMPUTER,
        &msg,
        1,
        MAV_COMP_ID_CAMERA,
        MAV_CMD_SET_CAMERA_ZOOM,
        0,
        CAMERA_ZOOM_TYPE::ZOOM_TYPE_RANGE,
        range,
        1, 0, 0, 0, 0);
    port->write_message(msg);
    cout << "Set camera zoom range to " << range << endl;

    time_t start = time(nullptr);
    bool got_respond = false;
    while (running)
    {
        time_t now = time(nullptr);
        if (now - start > 3)
        {
            if (!got_respond)
                cerr << "Failed to receive CAMERA_SETTINGS" << endl;
            break;
        }

        if (port->read_message(msg))
        {
            if (msg.msgid == MAVLINK_MSG_ID_CAMERA_SETTINGS)
            {
                mavlink_camera_settings_t t;
                mavlink_msg_camera_settings_decode(&msg, &t);
                cout << "camera_device_id=" << (int)t.camera_device_id << " zoomLevel=" << t.zoomLevel << endl;
                got_respond = true;
            }
        }
    }
}

void get_gimabal_device_info(Port *port)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        1,
        MAV_COMP_ID_ONBOARD_COMPUTER,
        &msg,
        1,
        MAV_COMP_ID_GIMBAL,
        MAV_CMD_REQUEST_MESSAGE,
        0,
        MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION,
        0, 0, 0, 0, 0, 0);
    port->write_message(msg);
    time_t start = time(nullptr);
    while (running)
    {
        time_t now = time(nullptr);
        if (now - start > 3)
        {
            cerr << "Msg GIMBAL_DEVICE_INFORMATION failed" << endl;
            break;
        }

        if (port->read_message(msg))
        {
            if (msg.msgid == MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION)
            {
                cout << "GotMsg GIMBAL_DEVICE_INFORMATION: ";
                mavlink_gimbal_device_information_t t;
                mavlink_msg_gimbal_device_information_decode(&msg, &t);
                cout << "gimbal_device_id=" << (int)t.gimbal_device_id << " vendor_name=" << t.vendor_name << "model_name=" << t.model_name << endl;
                break;
            }
        }
    }
}

void set_gimbal_device_attitude(Port *port, float pitch, float yaw)
{
    mavlink_message_t msg;
    float q[4];
    mavlink_euler_to_quaternion(0, pitch, yaw, q);
    mavlink_msg_gimbal_device_set_attitude_pack(
        1,
        MAV_COMP_ID_ONBOARD_COMPUTER,
        &msg,
        1,
        MAV_COMP_ID_GIMBAL,
        GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME,
        q,
        NAN, NAN, NAN);
    port->write_message(msg);

    time_t start = time(nullptr);
    bool got_respond = false;
    while (running)
    {
        time_t now = time(nullptr);
        if (now - start > 3)
        {
            if (!got_respond)
                cerr << "Failed to receive GIMBAL_DEVICE_ATTITUDE_STATUS" << endl;
            break;
        }

        if (port->read_message(msg))
        {
            if (msg.msgid == MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS)
            {
                mavlink_gimbal_device_attitude_status_t t;
                mavlink_msg_gimbal_device_attitude_status_decode(&msg, &t);
                cout << "gimbal_device_id=" << (int)t.gimbal_device_id << " q=[" << t.q[0] << t.q[1] << t.q[2] << t.q[3] << "]" << endl;
                got_respond = true;
            }
        }
    }
}

int main(int argc, char **argv)
{
    CLI::App app{"Payload Test Controller"};
    string device_path;
    app.add_option("-d,--device", device_path, "Device path, e.g. /dev/ttyUSB0");
    string udp_ip;
    app.add_option("-u,--udp", udp_ip, "UDP IP address, e.g. 192.168.144.240");
    int udp_port = 14550;
    app.add_option("-p,--port", udp_port, "UDP port number, default: 14550");
    CLI11_PARSE(app, argc, argv);

    Port *port;
    if (!device_path.empty())
    {
        port = new Port(device_path.c_str(), B115200);
    }
    else if (!udp_ip.empty())
    {
        port = new Port(udp_ip.c_str(), udp_port);
    }
    else
    {
        cout << app.help();
        return 0;
    }

    signal(SIGINT, quit_handler);

    send_heartbeat(port);
    get_Camera_settings(port);
    set_camera_zoom_range(port, 100);
    set_camera_zoom_range(port, 0);
    get_gimabal_device_info(port);
    set_gimbal_device_attitude(port, -M_PI_4, M_PI_2);
    set_gimbal_device_attitude(port, 0, 0);

    delete port;

    return 0;
}
