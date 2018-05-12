from robocluster.manager.ProcessManager import ProcessManager, RunOnce


process_list = [
    RunOnce('USBManager', 'python3 USBmanager.py'),
     RunOnce('Printer', 'python3 Printer.py'),
    # RunOnce('GPSDriver', 'python3 GPSdriver.py'),
    # RunOnce('DriveControl', 'python3 DriveProcess.py'),
    # RunOnce('Joystick', 'python3 JoystickProcess.py'),

]


with ProcessManager() as manager:
    # Initialize all the processes
    for proc in process_list:
        manager.addProcess(proc)

    # Start all processes
    manager.start()

    try:
        # Run asyncio event loop
        manager.run()
    except KeyboardInterrupt:
        pass # exit cleanly
