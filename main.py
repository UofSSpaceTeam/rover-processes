from robocluster.manager.ProcessManager import ProcessManager, RunOnce
import time


process_list = [
    # RunOnce('USBManager', 'python3 USBmanager.py'),
    RunOnce('Printer', 'python3 Printer.py'),
    # RunOnce('GPSDriver', 'python3 GPSdriver.py'),
    # RunOnce('DriveControl', 'python3 DriveProcess.py'),
    RunOnce('Joystick', 'python3 JoystickProcess.py'),
    RunOnce('Arm', 'python3 ArmProcess.py')
    # RunOnce('getter', 'python3 usb_req_example.py'),

]


with ProcessManager() as manager:
    # Initialize all the processes
    for proc in process_list:
        manager.addProcess(proc)

    # Start all processes
    manager.start()

    try:
        # Run asyncio event loop
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass # exit cleanly
