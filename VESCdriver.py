from robocluster import SerialDriver, Device
from robocluster.ports.serial import SerialPort
from pyvesc import SetRPM

class VESCDriver(Device):
    """Device that exposes a serial device to the robocluster network."""

    def __init__(self, name, group, loop=None, disable_receive_loop=False):
        super().__init__(name, group, loop=loop)
        self.encoding = 'vesc'
        self.serial_port = SerialPort(name,
                encoding=self.encoding, loop=self._loop,
                disable_receive_loop=disable_receive_loop)
        self.serial_port.on_recv('send', self.handle_packet)
        self.serial_port.on_recv('publish', self.handle_packet)
        self.serial_port.on_recv('heartbeat', self.handle_packet)
        # self._router.on_message(self.forward_packet)
        self.serial_port.start()

    async def handle_packet(self, other, message):
        """Forward messages from serial to robocluster network."""
        print(message)
        if message.type == 'heartbeat':
            self.name = message.source
            self._router.name = self.name

    async def forward_packet(self, packet):
        """Forwards messages from robocluster network to serial device."""
        await self.serial_port.send(packet.to_json())

    async def write(self, data):
        """Write to the serial device."""
        #TODO: the message creation is wrong...
        if self.encoding == 'json':
            msg = Message(self.name, 'publish', data)
        elif self.encoding == 'vesc':
            msg = data
        await self.serial_port.send(msg)


driver = VESCDriver('/dev/ttyACM0', 'rover')
drive_device = Device('drive', 'rover')

@driver.every('1s')
async def print_name():
    print(driver.name)

@driver.on('*/SetRPM')
async def setrpm(event, data):
    print(event, data)
    await driver.write(SetRPM(int(data)))

@drive_device.every('100ms')
async def rpm():
    await drive_device.send('wheelLF', 'SetRPM', 1500)

try:
    driver.start()
    drive_device.start()
    driver.wait()
    drive_device.wait()
except KeyboardInterrupt:
    driver.stop()
    drive_device.stop()
