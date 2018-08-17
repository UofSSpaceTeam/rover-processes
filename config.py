# Put constants/variables all processes need to share in here
import logging
import inspect
import time
import os

# The network for robocluster Devices to use.
# For local testing on your laptop, 0.0.0.0/0 should work,
# for the network on the rover, 10.0.0.0/24 is the one you need.
network = '10.0.0.0/24'
# network = '0.0.0.0/0'

# Which xbox controller controlls which device.
# Typically you want to just plug in one controller
# to test a single module, so say you want to test the drill,
# set `drill_controller = 0`. If two processes are running
# while configured to use the same controller, that controller
# will control both processes, so if the rover starts driving
# when you go to move the arm, that's the first thing to check.
drive_controller = 0
arm_controller = 1
drill_controller = 1
front_end_loader_controller = 1

# Defines the logging level of each process or python module.
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
    'JSONDriver': default_level
}

# A filter string to filter logs by. see the official documentation
# for the python logging module to see what a valid format is.
log_filter_string = ''

if not os.path.isdir('./logs'):
    os.makedirs('logs')
# Day-Month(abbrviation)-Year:24Hour:Minute
time_string = time.strftime('%d-%b-%Y:%H:%M')
logfile_path = './logs/{}.log'.format(time_string)

def getLogger():
    """
    This uses some python hacks to retrieve the name of the python
    script that calls getLogger and produces a Logger object with it.
    For example, if you call getLogger from ArmProcess.py, it will return
    a Logger object with the name "ArmProcess"
    """
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
