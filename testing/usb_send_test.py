from robocluster import Device


d = Device('Device', 'rover')


@d.every('1s')
async def send_msg():
    await d.publish('ExampleDevice', {'test':42})

d.start()
d.wait()
