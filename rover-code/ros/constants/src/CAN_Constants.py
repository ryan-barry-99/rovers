from enum import Enum

CHANNEL = {
    "JETSON": 0,
    "MAIN_BODY": 1,
    "SCIENCE_BOARD": 2,
    "ARM_BOARD": 3
}


TOPICS = {
    0: {
        "id": 0,
        "name": "E_STOP",
        "buf": bytearray(8),
        "channel": CHANNEL["MAIN_BODY"],
    },
    1: {
        "id": 1,
        "name": "TARGET_VELOCITY",
        "buf": bytearray(8),
        "channel": CHANNEL["MAIN_BODY"],
    },
    2: {
        "id": 2,
        "name": "CURRENT_VELOCITY",
        "buf": bytearray(8),
        "channel": CHANNEL["MAIN_BODY"],
    },
    3: {
        "id": 3,
        "name": "IMU",
        "buf": bytearray(8),
        "channel": CHANNEL["MAIN_BODY"],
    }
}

def get_id(id):
    for key, value in TOPICS.items():
        if value["name"] == id:
            return key