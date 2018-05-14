from robocluster import Device

from roverutil import getnetwork

'''
Just prints everything it published from the robocluster network.
'''

printer = Device('printer', 'rover', network=getnetwork())

@printer.on('*/*')
def print_it(event, data):
    print(event, data)


printer.start()
printer.wait()
