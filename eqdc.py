#! /bin/env python
"""
Equipment Servicing Diagnostic client.
Used for interacting with the Equipment Servicing Diagnostic system
for the Equipment Servicing task in CIRC 2018.
"""

import argparse
import socket

# HOST = 'task.cstag.ca'
HOST = '192.168.0.1'
PORT = 4547
BUFF_SIZE = 2048

def parse_args():
    parser = argparse.ArgumentParser(description='Interact with the CIRC Equipment servicing diagnostic system')
    parser.add_argument('command')
    parser.add_argument('arguments', nargs='*')

    args = parser.parse_args()
    api = API()

    if args.command:
        function = getattr(api, args.command)
        if function:
            response = function(*args.arguments)
            print(response)
        else:
            print('Invalid command')

class API:
    """
    To add support for new commands, create a new method
    with the same name you want to have entered from the
    command line. CLI arguments after the command name
    will be passed in to your method in the order they were
    entered on the CLI.
    """

    def help(self):
        return self._send_mesg('help')

    def status(self):
        return self._send_mesg('status')

    def login(self, username, password):
        return self._send_mesg('login {} {}'.format(username, password))

    def logout(self):
        return self._send_mesg('logout')

    def start(self):
        return self._send_mesg('start')

    def stop(self):
        return self._send_mesg('stop')

    def _send_mesg(self, msg):
        client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client.sendto(msg.encode(), (HOST, PORT))
        response, addr = client.recvfrom(BUFF_SIZE)
        return response.decode('utf-8')

parse_args()
