# Put constants/variables all processes need to share in here
import logging
import inspect
import time
import os

# network = '10.0.0.0/24'
network = '0.0.0.0/0'

drive_controller = 0
arm_controller = 1
drill_controller = 0

default_level = logging.DEBUG
level_map = {
    'Printer': default_level,
    'Autopilot': default_level,
    'USBmanager': default_level,
    'JoystickProcess': logging.INFO,
    'NavigationProcess': default_level,
    'DriveProcess': default_level,
    'KalmanFilterProcess': default_level,
    'GPSDriver': default_level,
    'ArmProcess': default_level,
    'Avoiding': default_level,
    'JSONDriver':logging.warning,
}

log_filter_string = ''

if not os.path.isdir('./logs'):
    os.makedirs('logs')
# Day-Month(abbrviation)-Year:24Hour:Minute
time_string = time.strftime('%d-%b-%Y:%H:%M')
logfile_path = './logs/{}.log'.format(time_string)

def getLogger():
    frame = inspect.stack()[1]
    name = frame.filename[:-3] # strip off .py extension
    logger = logging.getLogger(name)
    formatter = logging.Formatter('%(asctime)s | %(name)-10s| %(levelname)-8s| %(message)s')
    log_filter = logging.Filter(log_filter_string)

    # Console logging
    console_log = logging.StreamHandler()
    console_log.setFormatter(formatter)
    console_log.setLevel(default_level)
    console_log.addFilter(log_filter)

    # File logging
    file_log = logging.FileHandler(logfile_path)
    file_log.setFormatter(formatter)
    file_log.setLevel(logging.DEBUG)
    # file_log.addFilter(log_filter)

    logger.addHandler(console_log)
    logger.addHandler(file_log)

    if name in level_map: # should this maybe be just for console log?
        level = level_map[name]
    else:
        level = default_level
    logger.setLevel(level)

    return logger
