import numpy as np

NUM_ACTORS = 3
K_CLUSTERS = 3
N_PREDICTIONS = 0
N_PREDICTIONS_DELTA = 0.1
CBF_RHO = 0.8
CBF_DS = 0.5
CBF_RADIUS = (CBF_RHO + CBF_DS)

CBF_ALPHA = 0.5
TIAGO_A = 0.25
TARGET_POINT = [5,5]
FAKE_FSM = True
CROWD_PRINT = False
DEBUG_PRINT = False
MOTION_PRINT = False
PROJECTION_MODE = True

MAXV = 1.0
MAXV_NEG = -0.2

DEG2RAD =  0.01745
MAXW =  DEG2RAD * 120
MAXW_NEG = -DEG2RAD  * 120
TRAJECTORY_SAVE = True

REGULATION_K1 = 0.2
REGULATION_K2 = 0.2
REGULATION_K3 = 0.2

KALMAN_NULLSTATE = np.array([999,999,999,999])
INNOVATION_THRESHOLD = 1
START_DIST_THRESHOLD = 0.1

TASK_TYPE = '8'

ABS_VALUE_TOO_CLOSE = 0.05


MAX_PRED_TIME = 1

SHOW_REAL_PPL = True