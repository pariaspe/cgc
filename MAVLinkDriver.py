import serial
import time
from pymavlink import mavutil

RATE = 50

TIMEOUT = 1.5
MISSION_TIMEOUT = 0.25
MAX_RETRIES = 5


def uav_connect(port, baudrate=None):
    """
    Creates a mavlink port which is connected with the airplane
    :param port: end point (type:IP:port)
    :param baudrate: transmission speed (baud rate)
    :return: mavlink port
    """

    # connect to the APM
    try:
        master = mavutil.mavlink_connection(port, baudrate, autoreconnect=True)  # retries=5
    except serial.serialutil.SerialException:
        print("[MAVLink Driver] Impossible connect to {}.".format(port))
        return None

    print('[MAVLink Driver] Connection established to device.')
    heartbeat = master.wait_heartbeat(blocking=True, timeout=TIMEOUT)
    if heartbeat is not None:
        print("[MAVLink Driver] Heartbeat Received: ", heartbeat)
    else:
        print("[MAVLink Driver] Heartbeat NOT received, bad connection.")

    if master.mavlink20():
        print('[MAVLink Driver] MAVLink version: 2.0')
    else:
        if master.mavlink10():
            print('[MAVLink Driver] MAVLink version: 1.0')
        else:
            print('[MAVLink Driver] MAVLink version: 0.9')

    # master.auto_mavlink_version()
    # version_handshaking(master)

    # Set the complete set of commands
    master.mav.request_data_stream_send(master.target_system,
                                        master.target_component,
                                        mavutil.mavlink.MAV_DATA_STREAM_ALL,
                                        RATE, 1)

    return master


def heartbeat_handler(master, status, stop_event):
    last_sent_heartbeat = 0
    while not stop_event.isSet():
        if master == 0:
            status[0] = 0
            continue

        if time.time() - last_sent_heartbeat > 0.5:
            master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            last_sent_heartbeat = time.time()

        last_received_heartbeat = master.time_since('HEARTBEAT')
        if last_received_heartbeat > 5:
            # print("[MAVLink Driver] Trying to reconnect.")
            status[0] = 2
        else:
            status[0] = 1

    status[0] = 0


def get_mission_item(system, component, *args):
    data = args[0]
    item = mavutil.mavlink.MAVLink_mission_item_message(system, component,
                                                        int(data[0]), int(data[2]), int(data[3]), int(data[1]), int(data[11]),
                                                        float(data[4]), float(data[5]), float(data[6]), float(data[7]),
                                                        float(data[8]), float(data[9]), float(data[10]))
    return item


def mission_item_to_text(item):
    str_item = str(item.seq) + "\t" + str(item.current) + "\t" + str(item.frame) + "\t" + str(item.command) + \
               "\t" + str(item.param1) + "\t" + str(item.param2) + "\t" + str(item.param3) + "\t" + \
               str(item.param4) + "\t" + str(item.x) + "\t" + str(item.y) + "\t" + str(item.z) + "\t" + \
               str(item.autocontinue) + "\n"
    return str_item


def reset_mission(master):
    """
    Implementation of clearing missions MAVLink Protocol
    :param master: mavlink connection
    :return: 0 on success, 1 otherwise
    """
    confirmation = 0
    clear = None
    while clear is None:
        master.mav.mission_clear_all_send(master.target_system, master.target_component)  # MV 1.0
        clear = master.recv_match(type=['MISSION_ACK'], blocking=True, timeout=MISSION_TIMEOUT)
        confirmation += 1
        if confirmation > MAX_RETRIES:
            break

    if clear is None:
        print('[MAVLink Driver] Mission couldnt be cleared.')
        return 1

    if getattr(clear, 'type') == 0:
        return 0
    else:
        return 1


def set_px4_mission(master, mission):
    """
    Implementation of uploading a mission to the vehicle by MAVLink Protocol
    Inspired by Colorado University Boulder Code: http://www.colorado.edu/recuv/2015/05/25/mavlink-protocol-waypoints
    :param master: mavlink connection
    :param mission: Mission class to upload
    :return: 0 on success, 1 otherwise
    """

    if master.mavlink20():
        version = 2
    else:
        version = 1

    version = 1  # force mavlink1
    reset_mission(master)

    print("[MAVLink Driver] " + str(len(mission)) + " mission items to send")
    if version == 2:
        master.mav.mission_count_send(master.target_system, master.target_component, len(mission), 0)
    else:
        master.mav.mission_count_send(master.target_system, master.target_component, len(mission))

    msg = None
    retry = 0
    while retry < MAX_RETRIES*10:
        msg = master.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'], blocking=True, timeout=MISSION_TIMEOUT)
        if msg is None:
            if version == 2:
                master.mav.mission_count_send(master.target_system, master.target_component, len(mission), 0)
            else:
                master.mav.mission_count_send(master.target_system, master.target_component, len(mission))
        else:
            print(msg)
            if msg.get_type() == 'MISSION_ACK' and msg.type != 0:
                msg = None
            if msg.get_type() == 'MISSION_ACK' and msg.type == 0:
                continue
            break
        retry += 1

    if msg is None:
        print('[MAVLink Driver] Mission uploading fail. Mission count not received.')
        return 1
    else:
        seq = int(msg.seq)

    retry = 0

    while retry < MAX_RETRIES*10:
        print('[MAVLink Driver] Sending waypoint {0} '.format(seq) + format(mission[seq]))
        master.mav.send(mission[seq])
        msg = master.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'], blocking=True, timeout=MISSION_TIMEOUT)

        if msg is not None:
            print(msg)
            if msg.get_type() == 'MISSION_ACK':
                mission_validation = msg
                if seq == len(mission)-1:
                    break
                else:
                    if getattr(msg, 'type') != 0:
                        break
                    continue

            seq = msg.seq
            retry = 0
        else:
            retry += 1
            print("Retry", retry)

    if msg is None:
        print('[MAVLink Driver] Mission uploading fail. Max retries limit reached.')
        return 1

    print("[MAVLink Driver] Mission ACK message (type = 0 means successful)" + str(mission_validation))

    if getattr(mission_validation, 'type') == 0:
        print('[MAVLink Driver] Mission SENDED')
        return 0
    else:
        return 1


def get_px4_mission(master):
    """
    Implementation of downloading a mission of the vehicle by MAVLink Protocol
    :param master: mavlink connection
    :return: mission at the vehicle as a Mission class
    """

    mission = []

    mission_request_list = mavutil.mavlink.MAVLink_mission_request_list_message(master.target_system,
                                                                                master.target_component)

    retry = 0
    while retry < MAX_RETRIES*10:
        master.mav.send(mission_request_list)
        count = master.recv_match(type=['MISSION_COUNT', 'MISSION_ACK'], blocking=True, timeout=MISSION_TIMEOUT)
        # print(retry, count)
        if count is not None:
            if count.get_type() == 'MISSION_ACK' and count.type != 0:
                count = None
            if count.get_type() == 'MISSION_ACK' and count.type == 0:
                continue
            print(count)
            break
        retry += 1

    print(count)
    if count is None:
        print("[MAVLink Driver] Mission download failed. Count not received.")
        return 1

    count = getattr(count, "count")
    print("[MAVLink Driver] Getting {0} mission items..".format(count))

    seq = 0
    retry = 0
    mission_item = None
    while seq < count:
        if retry > MAX_RETRIES*10:
            mission_item = None
            break
        mission_request = mavutil.mavlink.MAVLink_mission_request_int_message(master.target_system, master.target_component,
                                                                              seq)
        master.mav.send(mission_request)

        mission_item = master.recv_match(type=['MISSION_ITEM', 'MISSION_ITEM_INT', 'MISSION_ACK'], blocking=True, timeout=MISSION_TIMEOUT)
        # print(retry, mission_item)
        if mission_item is not None:
            if mission_item.get_type() == 'MISSION_ACK' and count.type != 0:
                mission_item = None
                break
            if mission_item.get_type() == 'MISSION_ACK' and count.type != 0:
                continue

            if seq == getattr(mission_item, "seq"):
                mission.append(mission_item)

            retry = 0

            print(mission_item)
            print("[MAVLink Driver] Mission item {0} received.".format(getattr(mission_item, "seq")))
            seq = getattr(mission_item, "seq") + 1
        else:
            retry += 1

    if mission_item is None:
        print("[MAVLink Driver] Mission download failed. Count not received.")
        return 1

    print("[MAVLink Driver] Mission download completed.")

    mission_ack = mavutil.mavlink.MAVLink_mission_ack_message(master.target_system, master.target_component, 0)
    master.mav.send(mission_ack)

    return mission
