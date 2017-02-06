import sys,tty,termios
class _Getch:       
    def __call__(self):
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

def get():
    inkey = _Getch()
    while(1):
            k=inkey()
            if k == 'c': print "C!"
            if k!='':
                print k
                break
    print 'you pressed', ord(k)

def main():
    for i in range(0,25):
        get()

if __name__=='__main__':
    main()