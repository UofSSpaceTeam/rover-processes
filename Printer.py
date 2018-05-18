from robocluster import Device
import config

'''
Just prints everything it published from the robocluster network.
'''

printer = Device('printer', 'rover', network=config.network)

@printer.on('*/*')
def print_it(event, data):
    print(event, data)


printer.start()
printer.wait()
