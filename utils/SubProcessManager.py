import os
import signal
import subprocess
import shlex

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
            self.subprocesses[name].kill()

            if self.subprocesses[name].wait() != 0:
                print("There were errors when closing" + name + " process")
            else:
                del self.subprocesses[name]

    def is_subprocess_running(self, name):
        return name in self.subprocesses and self.subprocesses[name].poll() is None

    # def update(self):
    #     names_to_delete = []
    #     for name in self.subprocesses:
    #         if self.subprocesses[name].poll() is not None:
    #             names_to_delete.append(name)

        # for name in names_to_delete:
        #     del self.subprocesses[name]
