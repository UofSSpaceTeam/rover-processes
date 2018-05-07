from robocluster.manager import ProcessManager, RunOnce
import time

network=None

process_list = [
    # RunOnce('USBManager', 'python3 USBmanager.py {}'.format(network)),
    # RunOnce('Printer', 'python3 Printer.py {}'.format(network)),
    # RunOnce('GPSDriver', 'python3 GPSdriver.py {}'.format(network)),
    RunOnce('DriveControl', 'python3 DriveProcess.py {}'.format(network)),
    # RunOnce('Joystick', 'python3 JoystickProcess.py {}'.format(network)),
    # RunOnce('KalmanFilter', 'python3 KalmanFilterProcess.py {}'.format(network)),
    RunOnce('Autopilot', 'python3 Autopilot.py {}'.format(network)),
    RunOnce('WebUI', 'python3 server.py {}'.format(network), cwd='rover-webui'),
    RunOnce('Navigation', 'python3 NewNavigationProcess.py {}'.format(network)),
    RunOnce('Simulator', 'python3 Simulator.py {}'.format(network)),
]


with ProcessManager(network=network) as manager:
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
