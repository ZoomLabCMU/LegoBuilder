import numpy as np

STUD_HEIGHT = 2
BRICK_WIDTH = 8
BRICK_HEIGHT = 9.6

class Brick:
    def __init__(self, color='GREY'):
        self.position = np.array([0, 0, 0]) #mm
        self.orientation = np.array([0, 0, 0, 0]) #quaternion

        #Bounding boxes for now
        self.bb_width = 0
        self.bb_length = 0
        self.bb_height = 0

        self.brickID = None
        self.color = None


class Brick_2x4(Brick):
    def __init__(self, color='RED'):
        super().__init__()
        self.bb_width = 2 * BRICK_WIDTH
        self.bb_length = 4 * BRICK_WIDTH
        self.bb_height = BRICK_HEIGHT

        self.brickID = 'Brick_2x4'
        self.color = color

