# Simulation parameters
ITERS_PER_STEP = 100
TIMESTEP = 0.01

# Stability threshold
Z_STABLE_THRESHOLD = 0.58  # If the Z pos of ANYmal base falls below, indicates a fall (unstable)

# Control dictionary (joint mapping)
CONTROL_DICT = {  # Use ALL CAPS since it's a constant
    'LF_HAA': 0,
    'LF_HFE': 1,
    'LF_KFE': 2,
    'RF_HAA': 3,
    'RF_HFE': 4,
    'RF_KFE': 5,
    'LH_HAA': 6,
    'LH_HFE': 7,
    'LH_KFE': 8,
    'RH_HAA': 9,
    'RH_HFE': 10,
    'RH_KFE': 11,
    'motor11p': 12,
    'motor12p': 13,
    'motor13p': 14
}
