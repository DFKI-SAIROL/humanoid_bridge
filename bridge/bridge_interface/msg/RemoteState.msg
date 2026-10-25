# The operator's remote, decoded by the bridge. Names as in the bridge YAML's estop combos.
bool connected      # the robot reports a remote in this message
string[] pressed    # buttons held now
float32 lx          # sticks, -1 to 1, as the robot reports them
float32 ly
float32 rx
float32 ry
