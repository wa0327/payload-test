#include "CLI11.hpp"
#include "port.hpp"
#include <iostream>
#include <csignal>
#include <optional>

using namespace std;

bool running = true;
optional<uint8_t> autopilot_id;
optional<uint8_t> camera_id;
optional<uint8_t> gimbal_id;

void quit_handler(int)
{
    running = false;
}

void send_heartbeat(Port *port)
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        1,
        MAV_COMP_ID_ONBOARD_COMPUTER,
        &msg,
        MAV_TYPE_ONBOARD_CONTROLLER,
        MAV_AUTOPILOT_INVALID,
        0, 0, MAV_STATE_ACTIVE);
    port->write_message(msg);
    cout << "Sent HEARTBEAT" << endl;

    time_t start = time(nullptr);
    bool got_respond = false;
    while (running)
    {
        time_t now = time(nullptr);
        if (now - start > 1)
        {
            if (!got_respond)
                cerr << "Cannot receive HEARTBEAT !!" << endl;
            break;
        }

        if (port->read_message(msg))
        {
            if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT)
            {
                mavlink_heartbeat_t t;
                mavlink_msg_heartbeat_decode(&msg, &t);
                cout << "Recv HEARTBEAT " << (int)msg.compid << " autopilot=" << (int)t.autopilot << " type=" << (int)t.type << " version=" << (int)t.mavlink_version << endl;
                got_respond = true;

                if (msg.compid == MAV_COMP_ID_AUTOPILOT1)
                {
                    if (not autopilot_id)
                    {
                        cout << "AutoPilot found" << endl;
                        autopilot_id = msg.compid;
                    }
                }
                else if (msg.compid == MAV_COMP_ID_CAMERA || msg.compid == MAV_COMP_ID_CAMERA2 || msg.compid == MAV_COMP_ID_CAMERA3 || msg.compid == MAV_COMP_ID_CAMERA4 || msg.compid == MAV_COMP_ID_CAMERA5 || msg.compid == MAV_COMP_ID_CAMERA6)
                {
                    if (not camera_id)
                    {
                        cout << "Camera found" << endl;
                        camera_id = msg.compid;
                    }
                }
                else if (msg.compid == MAV_COMP_ID_GIMBAL || msg.compid == MAV_COMP_ID_GIMBAL2 || msg.compid == MAV_COMP_ID_GIMBAL3 || msg.compid == MAV_COMP_ID_GIMBAL4 || msg.compid == MAV_COMP_ID_GIMBAL5 || msg.compid == MAV_COMP_ID_GIMBAL6)
                {
                    if (not gimbal_id)
                    {
                        cout << "Gimbal found" << endl;
                        gimbal_id = msg.compid;
                    }
                }
            }
        }
    }
}

void get_camera_info(Port *port, uint8_t camera_id)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        1,
        MAV_COMP_ID_ONBOARD_COMPUTER,
        &msg,
        1,
        camera_id,
        MAV_CMD_REQUEST_MESSAGE,
        0,
        MAVLINK_MSG_ID_CAMERA_INFORMATION,
        0, 0, 0, 0, 0, 0);
    port->write_message(msg);
    cout << "Request CAMERA_INFORMATION" << endl;

    time_t start = time(nullptr);
    while (running)
    {
        time_t now = time(nullptr);
        if (now - start > 1)
        {
            cerr << "Cannot receive CAMERA_INFORMATION !!" << endl;
            break;
        }

        if (port->read_message(msg))
        {
            if (msg.msgid == MAVLINK_MSG_ID_CAMERA_INFORMATION)
            {
                mavlink_camera_information_t t;
                mavlink_msg_camera_information_decode(&msg, &t);
                cout << "Recv CAMERA_INFORMATION camera_device_id=" << (int)t.camera_device_id << " model_name=" << t.model_name << " resolution=" << t.resolution_h << "x" << t.resolution_v << endl;
                break;
            }
        }
    }
}

void get_camera_settings(Port *port, uint8_t camera_id)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        1,
        MAV_COMP_ID_ONBOARD_COMPUTER,
        &msg,
        1,
        camera_id,
        MAV_CMD_REQUEST_MESSAGE,
        0,
        MAVLINK_MSG_ID_CAMERA_SETTINGS,
        0, 0, 0, 0, 0, 0);
    port->write_message(msg);
    cout << "Sent REQUEST_CAMERA_SETTINGS" << endl;

    time_t start = time(nullptr);
    while (running)
    {
        time_t now = time(nullptr);
        if (now - start > 1)
        {
            cerr << "Cannot receive CAMERA_SETTINGS !!" << endl;
            break;
        }

        if (port->read_message(msg))
        {
            if (msg.msgid == MAVLINK_MSG_ID_CAMERA_SETTINGS)
            {
                mavlink_camera_settings_t t;
                mavlink_msg_camera_settings_decode(&msg, &t);
                cout << "Recv CAMERA_SETTINGS camera_device_id=" << (int)t.camera_device_id << " zoomLevel=" << t.zoomLevel << endl;
                break;
            }
        }
    }
}

void set_camera_zoom_range(Port *port, uint8_t camera_id, int range)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        1,
        MAV_COMP_ID_ONBOARD_COMPUTER,
        &msg,
        1,
        camera_id,
        MAV_CMD_SET_CAMERA_ZOOM,
        0,
        CAMERA_ZOOM_TYPE::ZOOM_TYPE_RANGE,
        range,
        1, 0, 0, 0, 0);
    port->write_message(msg);
    cout << "Sent CAMERA_ZOOM_RANGE " << range << endl;

    time_t start = time(nullptr);
    bool got_respond = false;
    while (running)
    {
        time_t now = time(nullptr);
        if (now - start > 1)
        {
            if (not got_respond)
                cerr << "Cannot receive CAMERA_SETTINGS !!" << endl;
            break;
        }

        if (port->read_message(msg))
        {
            if (msg.msgid == MAVLINK_MSG_ID_CAMERA_SETTINGS)
            {
                mavlink_camera_settings_t t;
                mavlink_msg_camera_settings_decode(&msg, &t);
                cout << "Recv CAMERA_SETTINGS camera_device_id=" << (int)t.camera_device_id << " zoomLevel=" << t.zoomLevel << endl;
                got_respond = true;
            }
        }
    }
}

void get_gimbal_manager_info(Port *port)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        1,
        MAV_COMP_ID_ONBOARD_COMPUTER,
        &msg,
        1,
        autopilot_id.value(),
        MAV_CMD_REQUEST_MESSAGE,
        0,
        MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION,
        0, 0, 0, 0, 0, 0);
    port->write_message(msg);
    cout << "Sent GIMBAL_MANAGER_INFORMATION" << endl;

    time_t start = time(nullptr);
    while (running)
    {
        time_t now = time(nullptr);
        if (now - start > 1)
        {
            cerr << "Cannot receive GIMBAL_MANAGER_INFORMATION !!" << endl;
            break;
        }

        if (port->read_message(msg))
        {
            if (msg.msgid == MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION)
            {
                mavlink_gimbal_manager_information_t t;
                mavlink_msg_gimbal_manager_information_decode(&msg, &t);
                cout << "Recv GIMBAL_MANAGER_INFORMATION gimbal_device_id=" << (int)t.gimbal_device_id << " cap_flags=" << t.cap_flags << endl;
                break;
            }
        }
    }
}

void set_gimbal_manager_attitude(Port *port, float pitch, float yaw)
{
    mavlink_message_t msg;
    float q[4];
    mavlink_euler_to_quaternion(0, pitch, yaw, q);
    mavlink_msg_gimbal_manager_set_attitude_pack(
        1,
        MAV_COMP_ID_ONBOARD_COMPUTER,
        &msg,
        1,
        autopilot_id.value(),
        GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME,
        0,
        q,
        NAN, NAN, NAN);
    port->write_message(msg);
    cout << "Sent GIMBAL_MANAGER_SET_ATTITUDE" << endl;

    time_t start = time(nullptr);
    bool got_respond = false;
    while (running)
    {
        time_t now = time(nullptr);
        if (now - start > 2)
        {
            if (!got_respond)
                cerr << "Cannot receive GIMBAL_DEVICE_ATTITUDE_STATUS !!" << endl;
            break;
        }

        if (port->read_message(msg))
        {
            if (msg.msgid == MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS)
            {
                mavlink_gimbal_device_attitude_status_t t;
                mavlink_msg_gimbal_device_attitude_status_decode(&msg, &t);
                if (msg.compid == MAV_COMP_ID_GIMBAL)
                {
                    float roll, pitch, yaw;
                    mavlink_quaternion_to_euler(t.q, &roll, &pitch, &yaw);
                    cout << "Recv GIMBAL_DEVICE_ATTITUDE_STATUS gimbal_device_id=" << (int)t.gimbal_device_id << " roll=" << roll / M_PI * 180 << " pitch=" << pitch / M_PI * 180 << " yaw=" << yaw / M_PI * 180 << "]" << endl;
                    got_respond = true;
                }
            }
        }
    }
}

void get_gimbal_device_info(Port *port)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        1,
        MAV_COMP_ID_ONBOARD_COMPUTER,
        &msg,
        1,
        gimbal_id.value(),
        MAV_CMD_REQUEST_MESSAGE,
        0,
        MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION,
        0, 0, 0, 0, 0, 0);
    port->write_message(msg);
    cout << "Sent REQUEST GIMBAL_DEVICE_INFORMATION" << endl;

    time_t start = time(nullptr);
    while (running)
    {
        time_t now = time(nullptr);
        if (now - start > 1)
        {
            cerr << "Cannot receive GIMBAL_DEVICE_INFORMATION !!" << endl;
            break;
        }

        if (port->read_message(msg))
        {
            if (msg.msgid == MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION)
            {
                mavlink_gimbal_device_information_t t;
                mavlink_msg_gimbal_device_information_decode(&msg, &t);
                cout << "Recv GIMBAL_DEVICE_INFORMATION gimbal_device_id=" << (int)t.gimbal_device_id << " vendor_name=" << t.vendor_name << "model_name=" << t.model_name << endl;
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
        gimbal_id.value(),
        GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME,
        q,
        NAN, NAN, NAN);
    port->write_message(msg);
    cout << "Sent GIMBAL_DEVICE_SET_ATTITUDE" << endl;

    time_t start = time(nullptr);
    bool got_respond = false;
    while (running)
    {
        time_t now = time(nullptr);
        if (now - start > 2)
        {
            if (!got_respond)
                cerr << "Cannot receive GIMBAL_DEVICE_ATTITUDE_STATUS !!" << endl;
            break;
        }

        if (port->read_message(msg))
        {
            if (msg.msgid == MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS)
            {
                mavlink_gimbal_device_attitude_status_t t;
                mavlink_msg_gimbal_device_attitude_status_decode(&msg, &t);
                if (msg.compid == MAV_COMP_ID_GIMBAL)
                {
                    float roll, pitch, yaw;
                    mavlink_quaternion_to_euler(t.q, &roll, &pitch, &yaw);
                    cout << "Recv GIMBAL_DEVICE_ATTITUDE_STATUS gimbal_device_id=" << (int)t.gimbal_device_id << " roll=" << roll / M_PI * 180 << " pitch=" << pitch / M_PI * 180 << " yaw=" << yaw / M_PI * 180 << "]" << endl;
                    got_respond = true;
                }
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
    if (camera_id)
    {
        get_camera_info(port, camera_id.value());
        get_camera_settings(port, camera_id.value());
        set_camera_zoom_range(port, camera_id.value(), 100);
        set_camera_zoom_range(port, camera_id.value(), 0);
    }
    else
    {
        get_camera_info(port, 0);
        get_camera_settings(port, 0);
        set_camera_zoom_range(port, 0, 100);
        set_camera_zoom_range(port, 0, 0);
    }
    if (autopilot_id)
    {
        get_gimbal_manager_info(port);
        set_gimbal_manager_attitude(port, -M_PI_4, M_PI_2);
        set_gimbal_manager_attitude(port, 0, 0);
    }
    else if (gimbal_id)
    {
        get_gimbal_device_info(port);
        set_gimbal_device_attitude(port, -M_PI_4, M_PI_2);
        set_gimbal_device_attitude(port, 0, 0);
    }

    delete port;

    return 0;
}
