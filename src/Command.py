import numpy as np


class Command:
    """Stores movement command
    """

    def __init__(self):
        self.horizontal_velocity = np.array([0, 0])
        self.yaw_rate = 0.0
        self.height = -0.16
        self.pitch = 0.0
        self.roll = 0.0
        self.activation = 0
        
        self.hop_event = False # # becomes true when hop button is pressed, false immediately after
        self.trot_event = False # becomes true when trot button is pressed, false immediately after
        self.activate_event = False # becomes true when activate button is pressed, false immediately after