

window = curses.initscr()
window.nodelay(1)
ch = -1
while ch < 0:
    # put code continuously run here
    ch = window.getch()
print(ch)