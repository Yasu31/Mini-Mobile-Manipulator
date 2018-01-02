import subprocess
from threading import Thread

def myThread(i):
    subprocess.run(["omxplayer", str(i)+".mp3", "--vol", "800"], timeout=4)
def playSound(i):
    t=Thread(target=myThread, args=(i,))
    t.start()
