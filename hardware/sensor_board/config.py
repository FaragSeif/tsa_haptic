from micropython import const

# ////////////////////////
#    HARDWARE SETTINGS
# ////////////////////////

# --------------------
# Load cell amplifiers
# --------------------
AMP1_ADC_PIN = 'A1'
AMP2_ADC_PIN = 'A3'

# -------------------
# Quadrature encoders
# -------------------
# TIM 1:
#   CH_A = "A8"``
#   CH_B = "A9"
ENC1_TIM = const(1)
# TIM 8:
#   CH_A = "C6"
#   CH_B = "C7"
ENC2_TIM = const(8)

# ---------------------
# CAN bus and actuators
# ---------------------

CAN_CH = const(1)
MOT_ID1 = const(321)
MOT_ID2 = const(322)


# //////////////////////////
#      COMMUNICATION 
# //////////////////////////

# -----------------
#   SPI settings
# -----------------
SPI_CH = const(1)
SPI_PRESCALE = const(16)  # the baudrate is given
# SPI_BAUD = /SPI_PRESCALE
SPI_POL = const(0)
SPI_PH = const(1)
SPI_BITS = 8

# ----------
#    UART 
# ----------

UART_CH = 3
UART_BAUD = 921600
UART_BITS = 8
UART_PARITY = None
UART_STOP = 1
UART_TIMEOUT = 0

# -----
# Sizes 
# -----
CMD_SIZE = const(12)
REPLY_SIZE = const(32)

# ---------
# Protocol 
# ---------
GET_SENSORS = b"\xC1"
SET_SPEED = b"\xC2"
SET_TORQUE = b"\xC3"
SET_MOT_CMD = b"\xC4"
INITIALIZE = b"\xC5"
TURN_ON = b"\xC6"
TURN_OFF = b"\xC7"

# ///////
# Formats
# ///////
CMD_FORMAT = '<BBhhhhh'
STATE_FORMAT = '<BBIhhihhihhhhBB'