from robocluster import Device

'''
Just prints everything it published from the robocluster network.
'''

printer = Device('printer', 'rover')

@printer.on('*')
def print_it(event, data):
    print(event, data)


printer.run()
