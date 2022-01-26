import Robo as rb
# import Camera as cam
import numpy as np
import copy
import time
import Socket as sc
import math

def test():
     
    # Inicial joint configuration
    q0 = np.array([np.deg2rad(180), np.deg2rad(-80), np.deg2rad(75), np.deg2rad(0), np.deg2rad(0)])
    
    
    # target
    p1 = np.array([.2, .2, .22])
    p2 = np.array([-.25, -.2, .3])
    p3 = np.array([.22, .22, .22])
    
    q1 = rb.ik(q0, p1)
    q2 = rb.ik(q1, p2)
    q3 = rb.ik(q2, p3)

    if not(rb.reachable(q1)) or not(rb.reachable(q2)) or not(rb.reachable(q3)):
        print('Ponto fora do espaco de trabalho')
        exit()
    
    rb.move(q0, copy.deepcopy(q1), 90, 'release')
    time.sleep(1)
    rb.move(q1, copy.deepcopy(q2), 90, 'nothing')
    time.sleep(1)
    rb.move(q2, copy.deepcopy(q3), 0, 'catch')
    time.sleep(1)
    rb.homePosition(90, q3)


def main():
    
    c = input('\n[1] Socket \n[2] Teste \n> ')

    if c == '1': 
        sc.connection()
    elif c == '2':
        test()
    else:
        pass
    
if __name__ == "__main__":
    main()