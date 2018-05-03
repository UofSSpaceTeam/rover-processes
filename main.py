from robocluster.manager import ProcessManager, RunOnce
import time


process_list = [
    #RunOnce('USBManager', 'python3 USBmanager.py'),
    RunOnce('Printer', 'python3 Printer.py'),
    # RunOnce('GPSDriver', 'python3 GPSdriver.py'),
    #RunOnce('DriveControl', 'python3 DriveProcess.py'),
    #RunOnce('Joystick', 'python3 JoystickProcess.py'),
    RunOnce('Autopilot', 'python3 Autopilot.py'),
    RunOnce('WebUI', 'python3 server.py', cwd='rover-webui')
]


with ProcessManager() as manager:
    # Initialize all the processes
    for proc in process_list:
        manager.addProcess(proc)

    # Start all processes
    manager.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass # exit cleanly
