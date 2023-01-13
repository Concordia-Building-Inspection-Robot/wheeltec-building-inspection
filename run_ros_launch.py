import os
import signal
import subprocess
import shlex

import time

class SubProcessManager():
    def __init__(self):
        self.subprocesses = {}

    # return true if subprocess didn't exist already, false otherwise
    def create_new_subprocess(self, name, command):
        if name not in self.subprocesses:
            proc = subprocess.Popen(shlex.split(command))
            self.subprocesses[name] = proc
            return True
        else:
            return False

    def close_subprocess(self, name):
        if name in self.subprocesses:
            self.subprocesses[name].terminate()

            if self.subprocesses[name].wait() != name in self.subprocesses and self.subprocesses[name].poll() is None:
                print("There were errors when closing" + name + " process")
            else:
                del self.subprocesses[name]

    def is_subprocess_running(self, name):
        running = name in self.subprocesses and self.subprocesses[name].poll() is None
        if not running and name in self.subprocesses:
            del self.subprocesses[name]
        return running

proc_man = SubProcessManager()
proc_man.create_new_subprocess('test', 'roslaunch data_collection start_collection.launch topic:=rosout device_name:=meme home_directory:=/home/william')

time.sleep(5)

proc_man.close_subprocess('test')

print(proc_man.is_subprocess_running('test'))