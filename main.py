from robocluster_manager.ProcessManager import ProcessManager, RunOnce


process_list = [
    # RunOnce('USBManager', 'python USBmanager.py'),
    RunOnce('Printer', 'python Printer.py'),
    # RunOnce('GPSDriver', 'python GPSdriver.py'),
    RunOnce('DriveControl', 'python DriveProcess.py'),
    RunOnce('Joystick', 'python JoystickProcess.py'),

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
