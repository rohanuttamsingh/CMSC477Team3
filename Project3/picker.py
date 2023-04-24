


if __name__ == "__main__":
    # --- PROGRAM STARTUP ---
    # initialization stuff goes here

    # --- BEGIN EXECUTION/MOVEMENT ---
    # Loop:
    #   Picker starts at starting location
    #   Move to LEGO source zone
    #       - have from P1 -> Dijkstra's + move
    #   Reach in and pick up LEGO (may involve slight forward movement)
    #       - have from P2 -> NN to detect, then align and grab
    #   Step back if necessary, then travel to river
    #       - have from P1 -> Dijkstra's + move
    #   Position as close as desired to river
    #       - have from P2 -> move_to_river() [adjust thresholds]
    #   Drop the LEGO on the other side
    #       - have from last week
    #   Signal to other robot that LEGO is ready for pickup
    #       - have from P2 -> socket stuff
    #   Return to LEGO source zone and loop
    #       - have from P1 -> Dijkstra's + move

    # NOTE: throughout entire above loop, want to be constantly checking for
    # static (boxes) and dynamic (enemy bot) obstacles and updating our map
    # ==> consider delegating this to a separate thread to run concurrently?

    # NOTE 2: on obstacles - best to account for static obstacles by simply
    # adding to the map then recalculating Dijkstra's as needed; for the other
    # robot, we have options: either stop and wait for it to leave frame, or
    # attempt to maneuever around it



    pass