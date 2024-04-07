class StateMachine:
    """
    State machine for kitbot autonomous.
    """

    green_count = 10
    total_count = 15

    def __init__(self):
        # super().__init__(self)

        #######################
        ### Values to track ###
        #######################
        self.block_count = 0
        self.block_held = False
        self.cur_state = "spin"
        self.prev_state = "?"

        self.green_seen = False  # green block spotted?
        self.red_seen = False  # red block spotted?

        # TODO: should also keep a 3 min timer to stop the robot once time is up.
        self.game_time = 180  # seconds

    def state_callback(self):
        """
        SPIN --> FOLLOW --> GRAB --> COLLECT  ()
                                 --> PLACE    ()
        """

        spin, follow, grab, collect, place = 0, 1, 2, 3, 4

        #######################
        ###      SPIN       ###
        #######################
        if self.cur_state == spin:
            if self.block_count > self.total_count:
                # TODO: done, terminate
                stop()
            if (self.block_count < self.green_count and self.green_seen) or (
                self.block_count > self.green_count and self.red_seen
            ):
                self.cur_state = follow  # ?????????? not entirely sure why these have to be separate clauses

        #######################
        ###     FOLLOW      ###
        #######################
        if self.cur_state == follow:
            if self.block_held:
                self.cur_state = place  # im not actually sure what you wanted here
                self.block_count += 1  # done with another block
            else:
                self.cur_state = grab  # grab the block once in position
                self.block_held = True

            self.prev_state = follow

        #######################
        ###      GRAB       ###
        #######################
        if self.cur_state == grab:
            if (
                self.block_count < self.green_count
            ):  # still collecting green blocks right now
                self.cur_state = collect  # put it in the robot
            else:
                self.cur_state = place  # i think this is the internal stacking? 
                                        # or is it setting the block down

            self.block_held = False  # no longer holding block in the grippah
            self.prev_state = grab

        #######################
        ###   COLLECTION    ###
        #######################
        if self.cur_state == collect:
            # doin stuff
            if self.block_count < self.green_count:
                if self.green_seen or self.red_seen:
                    self.cur_state = follow  # or following
                else:
                    self.cur_state = spin

            self.prev_state = collect
