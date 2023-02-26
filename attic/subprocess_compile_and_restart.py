import psutil
import os
import shutil

def kill_processes(name):
    for proc in psutil.process_iter():
        n = proc.name()
        if (n == name):
            print(proc.name())

        try:
            if proc.name() == name:
                proc.kill()
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass


os.system("ninja -C build")

kill_processes("tracking_subprocess_copy.exe")

copied = False
for i in range(10):
    print("Copying")
    try:
        shutil.copy("build\\mercury\\bin\\win64\\tracking_subprocess.exe", "build\\mercury\\bin\\win64\\tracking_subprocess_copy.exe")
        copied = True
        break
    except PermissionError:
        print("Couldn't copy")
print("Copied: ", copied)
