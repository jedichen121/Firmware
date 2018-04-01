uint8 NUM_ACTUATOR_OUTPUTS      = 16
uint8 NUM_ACTUATOR_OUTPUT_GROUPS    = 4 # for sanity checking
uint32 noutputs             # valid outputs
float32[16] output          # output data, received from mavlink message, in range 0..1 and -1..1