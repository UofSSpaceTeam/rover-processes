import logging
from robocluster import Device
import config

'''
Just prints everything it published from the robocluster network.
'''

log = config.getLogger()

printer = Device('printer', 'rover', network=config.network)

@printer.on('*/*')
def print_it(event, data):
    log.info("{}: {}".format(event, data))


printer.start()
printer.wait()
