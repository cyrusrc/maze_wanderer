class ZigZagBot:
    """Simple container class that stores state about the TurtleBot.

    Attributes:
        left_scan (float): the value of the leftmost laserscan reading
        right_scan (float): the the value of the rightmost laserscan reading

        orientation (float): yaw value drawn from /odom topic -- has value between -180 degrees and 180 degrees
        target_orientation (float): yaw value that the bot turns toward when it executes a 90ish degree turn

        x (float): x-coordinate in absolute terms (relative to gazebo model world)
        y (float): y-coordinate in absolute terms (relative to gazebo model world)
        is_bumping (bool): whether the bot's bumper is currently triggered
        is_turning_toward_wall (bool): whether the bot is actively facing up to a wall
        is_making_90ish_turn (bool): whether the bot is currently executing a 90ish degree turn
        turn_90ish_direction (int): 1 indicates counterclockwise; -1 indicates clockwise
    """
    def __init__(self):
        self.left_scan = 0.0
        self.right_scan = 0.0

        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.target_orientation = 0.0

        self.is_bumping = False
        self.is_turning_toward_wall = False
        self.is_making_90ish_turn = False
        self.turn_90ish_direction = 0.0
