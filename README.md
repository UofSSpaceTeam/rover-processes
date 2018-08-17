# Rover Processes

This is all the software that runs on the rover (that we wrote our selves).

Programs that go here should be relatively self contained,
if you have multiple files, put them in a folder with the
program entry being clearly named. If you have more than
a few files, consider creating a library in a new
repository instead.

## Install/Setup
Make sure you have Python3 and pip3 installed, and do:
```
pip3 install -r requirements.txt
```
This should install all the third party packages. Some processes may have additional
software that needs to be installed to run. These processes are:
- YagiSdrProcess.py: Needs [gqrx](http://gqrx.dk/download/gqrx-sdr-for-the-raspberry-pi) for the Raspberry Pi.
- LoadCell.py: Needs the [libphidgets22](https://www.phidgets.com/docs/OS_-_Linux) library.

You will also need the [robocluster](https://github.com/UofSSpaceTeam/robocluster) library
installed. Instructions for that are on it's Github page. It is recomended that you clone
the source and install it with the `-e` flag, as it makes updating robocluster as easy
as just pulling from the robocluster repo.


## Running the software
To run the rover software, run the `main.py` script:
```
python3 main.py
```
This will start up all enabled processes. `Ctrl-C` will halt all processes.
There may still be issues with `Ctrl-C` not working on Windows, someone should figure
that out at somepoint :).
To changes which processes are enabled, open `main.py` and find the `process_list`
variable. Comment out entries in the list to disable processes, and uncomment them
to enable them. Adding new processes is done by simply adding a new `RunOnce` instance
to the list, just like the existing processes. Note that scripts need to be in the
same directory as `main.py`, if you need to run something that is stored somewhere else
(such as the web UI), use the `cwd` parameter to the `RunOnce` constructor.

More details about how the ProcessManager works can be found in the
[robocluster](https://robocluster.readthedocs.io/en/latest/?badge=latest) documentation.

## Configuration
Various settings can be found in `config.py`.
Any constants that multiple processes will use should go in `config.py`.
Descriptions of all options are documented in comments. Please update the comments
if you change the behaviour or add new settings.
Most important setting is `network`. If none of the processes seem to be talking
to each other, it is likely because `network` is not set properly.

