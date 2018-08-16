from robocluster.manager import ProcessManager, RunOnce
import time
import config

network=config.network

process_list = [
    # RunOnce('USBManager', 'python3 USBmanager.py'),
    # RunOnce('Printer', 'python3 Printer.py'),
    # RunOnce('GPSDriver', 'python3 GPSdriver.py'),
    # RunOnce('DriveControl', 'python3 DriveProcess.py'),
    RunOnce('Joystick', 'python3 JoystickProcess.py'),
    # RunOnce('KalmanFilter', 'python3 KalmanFilterProcess.py'),
    # RunOnce('Autopilot', 'python3 Autopilot.py'),
    RunOnce('WebUI', 'python3 server.py', cwd='rover-webui'),
    # RunOnce('Navigation', 'python3 NavigationProcess.py'),
    # RunOnce('Simulator', 'python3 Simulator.py'),
    #RunOnce('PiCam', 'bash pi_cam.sh front'),
    # RunOnce('ArmProcess', 'python3 ArmProcess.py'),
    #RunOnce('webcam', 'bash start_webcam.sh'),
    # RunOnce('ObstacleDetection', 'python3 Avoidance.py'),
    # RunOnce('LoadCell', 'python3 LoadCell.py')
    # RunOnce('ScienceDrill', 'python3 DrillProcess.py'),
    # RunOnce('FrontEndLoader', 'python3 FrontEndLoaderProcess.py')
    #RunOnce('ScienceControll', 'python3 NewScience.py'),
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
