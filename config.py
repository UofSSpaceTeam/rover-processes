# Put constants/variables all processes need to share in here
import logging

network = '0.0.0.0/0'

loglevel = logging.DEBUG

logfile_path = 'log.log'

def getLogger():
    logger = logging.getLogger()
    formatter = logging.Formatter('%(asctime)s | %(module)-10s| %(levelname)-8s| %(message)s')

    console_log = logging.StreamHandler()
    console_log.setFormatter(formatter)
    console_log.setLevel(logging.INFO)

    # File logging
    file_log = logging.FileHandler(logfile_path)
    file_log.setFormatter(formatter)
    file_log.setLevel(logging.DEBUG)

    logger.addHandler(console_log)
    logger.addHandler(file_log)
    logger.setLevel(loglevel)

    return logger
