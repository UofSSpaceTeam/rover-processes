# Put constants/variables all processes need to share in here
import logging
import inspect

network = '0.0.0.0/0'

default_level = logging.DEBUG
level_map = {
    'Printer': default_level,
    'Autopilot': default_level,
    'USBmanager': default_level,
    'JoystickProcess': default_level,
    'NavigationProcess': default_level,
    'DriveProcess': default_level,
    'KalmanFilterProcess': default_level,
    'GPSDriver': default_level,
}

log_filter_string = ''

logfile_path = 'log.log'

def getLogger():
    frame = inspect.stack()[1]
    name = frame.filename[:-3] # strip off .py extension
    logger = logging.getLogger(name)
    formatter = logging.Formatter('%(asctime)s | %(name)-10s| %(levelname)-8s| %(message)s')
    log_filter = logging.Filter(log_filter_string)

    # Console logging
    console_log = logging.StreamHandler()
    console_log.setFormatter(formatter)
    console_log.setLevel(logging.DEBUG)
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
