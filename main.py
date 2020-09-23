#!/usr/bin/env python3

import sys
import threading
from MAVLinkDriver import uav_connect, get_mission_item, heartbeat_handler, set_px4_mission

DEFAULT_MISSION = "mission.txt"
# DEFAULT_CONNECTION = "udp:127.0.0.1:14540"
DEFAULT_CONNECTION = "serial:/dev/ttyACM0"
DEFAULT_BAUDRATE = 57600

MASTER = None
STATUS = [0]
EVENT_KILL_HANDLER = threading.Event()


def read_mission_from_file(filename):
    items = []

    file = open(filename, 'r')
    text = file.read()
    for i, line in enumerate(text.split("\n")):
        if i == 0:
            continue
        elif line == "":
            break

        tokens = line.split("\t", 12)
        system = None
        component = None
        if MASTER is not None:
            system = MASTER.target_system
            component = MASTER.target_component

        items.append(get_mission_item(system, component, tokens))

    file.close()
    return items


def do_send(*args):
    data = args[0]
    if not data:
        mission_file = DEFAULT_MISSION
    else:
        mission_file = data[0]

    if MASTER is None:
        print("[Error] Establish connection first.", file=sys.stderr)
    items = read_mission_from_file(mission_file)
    result = set_px4_mission(MASTER, items)
    if result == 0:
        print("Mission sent correctly.")
    else:
        print("[Error] Mission sending failed.", file=sys.stderr)


def do_show(*args):
    data = args[0]
    if not data:
        mission_file = DEFAULT_MISSION
    else:
        mission_file = data[0]

    items = read_mission_from_file(mission_file)
    for item in items:
        print(item)


def do_connect(*args):
    data = args[0]
    if not data:
        connection = DEFAULT_CONNECTION
        baudrate = DEFAULT_BAUDRATE
    elif len(data) == 1:
        connection = data[0]
        baudrate = DEFAULT_BAUDRATE
    elif len(data) > 1:
        connection = data[0]
        baudrate = data[1]

    type_con = connection.split(":")[0]
    if type_con == "serial":
        connection = connection.split(":")[-1]

    master = uav_connect(connection, baudrate)
    if master is not None:
        handler = threading.Thread(target=heartbeat_handler, args=(
            master, STATUS, EVENT_KILL_HANDLER), name='heartbeat_handler')
        handler.start()

        global MASTER
        MASTER = master


def do_help(*args):
    data = args[0]
    if not data:
        print("Command Ground Control, version 1.0.0")
        print("Commands are defined internally. Type help' to si this list.")
        print("Type 'help cmd' to know more about the command 'cmd'.")
        print("Type 'help cgc' to know more about Command Ground Control in general.")
        print("")
        print("connect [connection] [baudrate]")
        print("help [cmd]")
        print("send [filename]")
        print("show [filename]")
    elif data[0] == "cgc":
        print("Command Ground Control, version 1.0.0")
        print("Author: Pedro Arias Perez")
        print("")
        print("Description: WORK IN PROGRESS")
    elif data[0] == "connect":
        print("connect: connect [connection] [baudrate]")
        print("WORK IN PROGRESS")
    elif data[0] == "help":
        print("help: help [cmd]")
        print("WORK IN PROGRESS")
    elif data[0] == "send":
        print("send: send [filename]")
        print("WORK IN PROGRESS")
    elif data[0] == "show":
        print("show: show [filename]")
        print("WORK IN PROGRESS")
    else:
        print("[Error] Invalid command. Please type 'help' to see available commands.", file=sys.stderr)


def process_line(s):
    tokens = s.split(" ")
    if tokens[0] == "help":
        do_help(tokens[1:])
    elif tokens[0] == "connect":
        do_connect(tokens[1:])
    elif tokens[0] == "show":
        do_show(tokens[1:])
    elif tokens[0] == "send":
        do_send(tokens[1:])
    elif tokens[0] == "":
        pass
    else:
        print("[Error] Invalid command. Please type 'help' to see available commands.", file=sys.stderr)


def main():
    while True:
        try:
            s = input("> ")
        except EOFError:
            break
        process_line(s)

    EVENT_KILL_HANDLER.set()


if __name__ == '__main__':
    main()
