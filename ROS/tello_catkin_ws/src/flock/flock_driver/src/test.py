
import sys, select, termios, tty

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    keys = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    for key in keys:
        if key == '\x03':
            raise KeyboardInterrupt
        elif key == '\x1a':
            exit()
    return keys

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    while True:
        keys = getKey()
        print(keys)