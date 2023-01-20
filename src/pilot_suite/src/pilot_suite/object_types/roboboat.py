<<<<<<< HEAD:src/arcturus_pilot/scripts/object_types.py
from enum import Enum

class ObjectType(Enum):
    CHANNEL_GREEN = 0,
    CHANNEL_RED = 1,
    AVOID_GREEN = 2,
    AVOID_RED = 3,
    SNACK_GREEN = 4,
    SNACK_RED = 5,
    SNACK_BLUE = 6,
    RETURN = 7,
    FIND_SEAT = 8,
    WATER_BLAST = 9,
    SKEEBALL = 10,
    OTHER = 11

def getObjectType(label, task_label):
    if task_label == 1:
        if label == 0:
            return ObjectType.CHANNEL_RED
        elif label == 1:
            return ObjectType.CHANNEL_GREEN
        else:
            return ObjectType.OTHER

    elif task_label == 2:
        if label == 0:
            return ObjectType.AVOID_RED
        elif label == 1:
            return ObjectType.AVOID_GREEN
        else:
            return ObjectType.OTHER

    elif task_label == 3 or label == 5:
        return ObjectType.FIND_SEAT

    if task_label == 4:
        if label == 0:
            return ObjectType.SNACK_RED
        elif label == 1:
            return ObjectType.SNACK_GREEN
        elif label == 3:
            return ObjectType.SNACK_BLUE
    
    if task_label == 5 or label == 7:
        return ObjectType.SKEEBALL

    if task_label == 6 or label == 6:
        return ObjectType.WATER_BLAST
    
    if task_label == 7 or label == 4:
        return ObjectType.RETURN
=======
from enum import Enum

class Label(Enum):
    RED_POLE = 0
    GREEN_POLE = 1
    WHITE_POLE = 2
    RED_BUOY = 3 
    GREEN_BUOY = 4
    YELLOW_BUOY = 5
    BLACK_BUOY = 6
    BLUE_BUOY = 7 
    DOCK = 8
    WATER_TARGET = 9
    SKEEBALL_TARGET = 10

class ObjectType(Enum):
    CHANNEL_GREEN = 0,
    CHANNEL_RED = 1,
    AVOID_GREEN = 2,
    AVOID_RED = 3,
    SNACK_GREEN = 4,
    SNACK_RED = 5,
    SNACK_BLUE = 6,
    RETURN = 7,
    FIND_SEAT = 8,
    WATER_BLAST = 9,
    SKEEBALL = 10,
    OTHER = 11

def getObjectType(label, task_label):
    if task_label == 1:
        if label == 0:
            return ObjectType.CHANNEL_RED
        elif label == 1:
            return ObjectType.CHANNEL_GREEN
        else:
            return ObjectType.OTHER

    elif task_label == 2:
        if label == 0:
            return ObjectType.AVOID_RED
        elif label == 1:
            return ObjectType.AVOID_GREEN
        else:
            return ObjectType.OTHER

    elif task_label == 3 or label == 5:
        return ObjectType.FIND_SEAT

    if task_label == 4:
        if label == 0:
            return ObjectType.SNACK_RED
        elif label == 1:
            return ObjectType.SNACK_GREEN
        elif label == 3:
            return ObjectType.SNACK_BLUE
    
    if task_label == 5 or label == 7:
        return ObjectType.SKEEBALL

    if task_label == 6 or label == 6:
        return ObjectType.WATER_BLAST
    
    if task_label == 7 or label == 4:
        return ObjectType.RETURN
>>>>>>> 351031eaa37bdb673668a5cb8e1ee8d050ae4dba:src/pilot_suite/src/pilot_suite/object_types/roboboat.py
