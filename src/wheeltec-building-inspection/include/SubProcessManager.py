import subprocess


class SubProcessManager():
    def __init__(self):
        self.subprocesses = {}

    def create_new_subprocess(self, name, command):
        proc = subprocess.Popen(command)
        self.subprocesses[name] = proc

    def close_subprocess(self, name):
        self.subprocesses[name].send_signal(subprocess.signal.SIGINT)

        if self.subprocesses[name].wait() != 0:
            print("There were errors when closing" + name + " process")
        else:
            del self.subprocesses[name]
