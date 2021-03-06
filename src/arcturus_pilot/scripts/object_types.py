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
