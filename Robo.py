import numpy as np
import time

import rcpy
import rcpy.motor as motor
import rcpy.encoder as encoder
import Adafruit_BBIO.GPIO as gpio

from AtMegaSerCon import send_command

import math

# lenght: Waist, Shoulder, Elbow, Wrist
l = np.array([.385, .220, .220, .155])



# forward kinematics
def fk(q):
    a = -l[3]*np.sin(q[1]+q[2]+q[3]) + l[2]*np.cos(q[1]+q[2]) + l[1]*np.cos(q[1])
    b = l[0] - l[3]*np.cos(q[1]+q[2]+q[3]) - l[2]*np.sin(q[1]+q[2]) - l[1]*np.sin(q[1])
    
    dhMatrix = np.matrix([[np.cos(q[0])*np.cos(q[1]+q[2]+q[3])*np.cos(q[4])+np.sin(q[0])*np.sin(q[4]), -np.cos(q[0])*np.cos(q[1]+q[2]+q[3])*np.sin(q[4])+np.sin(q[0])*np.cos(q[4]), -np.cos(q[0])*np.sin(q[1]+q[2]+q[3]), np.cos(q[0])*a] , 
        [np.sin(q[0])*np.cos(q[1]+q[2]+q[3])*np.cos(q[4])-np.cos(q[0])*np.sin(q[4]), -np.sin(q[0])+np.cos(q[1]+q[2]+q[3])*np.sin(q[4])-np.cos(q[0])*np.cos(q[4]), -np.sin(q[0])*np.sin(q[1]+q[2]+q[3]), np.sin(q[0])*a] ,
        [-np.sin(q[1]+q[2]+q[3])*np.cos(q[4]), np.sin(q[1]+q[2]+q[3])*np.sin(q[4]), -np.cos(q[1]+q[2]+q[3]), b],
        [0, 0, 0, 1]])

    return dhMatrix
    
    
    
# inverse kinematics
def ik(q0, p):
    # Rotacao do motor 1
    th1 = np.arctan2(p[1], p[0]);

    # Rotacao do motor 3
    th234 = np.arctan2(np.sin(q0[1]+q0[2]+q0[3]), np.cos(q0[1]+q0[2]+q0[3]));
    c3 = ((np.cos(th1)*p[0] + np.sin(th1)*p[1] + l[3]*np.sin(q0[1]+q0[2]+q0[3]))**2 + (p[2] - l[0] + l[3]*np.cos(q0[1]+q0[2]+q0[3]))**2 - l[1]**2 - l[2]**2)/(2*l[1]*l[2]);
    s3 = np.sqrt(1-c3**2);
    th3 = np.arctan2(s3, c3);

    # Rotacao do motor 2
    c2 = ((np.cos(th1)*p[0] + np.sin(th1)*p[1] + l[3]*np.sin(q0[1]+q0[2]+q0[3]))*(c3*l[2] + l[1]) - (p[2] - l[0] + l[3]*np.cos(q0[1]+q0[2]+q0[3]))*s3*l[2])/((c3*l[2] + l[1])**2 + s3**2*l[2]**2);
    s2 = -((np.cos(th1)*p[0] + np.sin(th1)*p[1] + l[3]*np.sin(q0[1]+q0[2]+q0[3]))*s3*l[2] + (p[2] - l[0] + l[3]*np.cos(q0[1]+q0[2]+q0[3]))*(c3*l[2] + l[1]))/((c3*l[2] + l[1])**2 + s3**2*l[2]**2);
    th2 = np.arctan2(s2, c2);

    # Rotacao do motor 4
    th4 = th234 - th2 - th3;
    
    return np.array([th1, th2, th3, th4, 0])

# Verifica se ponto e alcancavel
def reachable(qf):
    
    if math.isnan(qf[0]) or math.isnan(qf[1])  or math.isnan(qf[2])  or  math.isnan(qf[3]) or math.isnan(qf[4]):
        return False
    return True
    
    
# trajectory
def traj(qi, qf, n):
    q = np.zeros((n,5))
    qd = np.zeros((n,5))
    qdd = np.zeros((n,5))
    
    for t in range(0, n):
        q[t, :] = qi + (3/n**2)*(qf-qi)*t**2 - (2/n**3)*(qf-qi)*t**3
        qd[t, :] = (6/n**2)*(qf-qi)*(t-t**2)
        qdd[t, :] = (qf-qi)*((6/n**2) - (12/n**3)*t);
        
    return q, qd, qdd


def motorPos(posT, m):  # run motor (Pos Target, Motor)
    
    encoder.set(m, 0)
    
    rcpy.set_state(rcpy.RUNNING)
    
    v = 0.8 if m == 3 and posT > 0 else 1
    
    print('\nVelocidade {:.1f}'.format(v))   
    
    if posT >= 0:
        high2 = posT + posT * 0.3
        low2 = posT - posT * 0.3
        high = posT + posT * 0.05
        low = posT - posT * 0.05
    else:
        low2 = posT + posT * 0.3
        high2 = posT - posT * 0.3
        low = posT + posT * 0.05
        high = posT - posT * 0.05
    
    print('low: {:2f}, high: {:2f}'.format(low, high))
    
    
    while True:
        
        if low <= encoder.get(m) <= high:
            print('\nencoder: {:+0d}'.format(encoder.get(m)))
            return True

        if low2 <= encoder.get(m) <= high2:
            print('\nencoder: {:+0d}'.format(encoder.get(m)))
            if v > 0.4:
                print('Reduzindo a velocidade...')
                v = v - 0.05
            else:
                v = 0.4
            
        if m==4:
            v = -v
            
        if posT >= encoder.get(m):
            while posT >= encoder.get(m):
                
                ant = encoder.get(m)
                
                print("\rforward: {:2f}".format(encoder.get(m)), end='')
                motor.set(m, -v)
                encoder.get(m)
                time.sleep(0.1)
                motor.set_brake(m)
                motor.set_free_spin(m)
                if ant == encoder.get(m) and m == 4:
                    return True
                if encoder.get(m) == ant:
                    print('\n\nERROR!\n')
                    exit()
        elif posT < encoder.get(m):
            while posT < encoder.get(m):
                
                ant = encoder.get(m)
                
                motor.set(m, v) 
                print("\rback: {:2f}".format(encoder.get(m)), end='')
                time.sleep(0.1)
                motor.set_brake(m)
                motor.set_free_spin(m)
                if encoder.get(m) == ant:
                    print('\n\nERROR!\n')
                    exit()
                if ant == encoder.get(m) and m != 4:
                    v = v + 0.1
                    print(m)
                elif ant == encoder.get(m) and m == 4:
                    return True
    
    rcpy.exit()
    

def move(qi, qf, roll, gripper):
    
    # angular displacement
    dq = np.zeros(5)
    
    # base
    if qf[0]<0:
        qf[0] = np.deg2rad(360)+qf[0]
    if qi[0]<0:
        qi[0] = np.deg2rad(360)+qi[0]
        
    dq[0] = abs(abs(qi[0])-abs(qf[0]))
    
    if qf[0]<qi[0]:
        dq[0] = -1*dq[0]
    
   
    # shoulder
    dq[1] = qf[1]-qi[1]
    #elbow
    dq[2] = qf[2] - (qi[2] - dq[1])
    # wrist
    dq[3] = qf[3] + dq[2]
    
    # encoders ticks: Base, Shoulder, Elbow, Wrist
    e_base = 14200*dq[0]/np.deg2rad(310)
    e_shoulder = 2000*dq[1]/np.deg2rad(50)
    e_elbow = 6000*dq[2]/np.deg2rad(160)
    e_wrist_pitch = 260*dq[3]/np.deg2rad(260)
    e_wrist_roll = 540*dq[4]/np.deg2rad(360)

    print('\n**************************************************************************')
    print('Posicao Inicial\tPosicao Final\t Deslocamento angular\t Pulsos de encoder')
    print('qi({:.4f}) \tqf({:.4f}) \t dq({:.4f}) \t\t e1({:.4f})'.format(np.rad2deg(qi[0]), np.rad2deg(qf[0]), np.rad2deg(dq[0]), e_base))
    print('qi({:.4f}) \tqf({:.4f}) \t dq({:.4f}) \t\t e2({:.4f})'.format(np.rad2deg(qi[1]), np.rad2deg(qf[1]), np.rad2deg(dq[1]), e_shoulder))
    print('qi({:.4f}) \tqf({:.4f}) \t dq({:.4f}) \t\t e3({:.4f})'.format(np.rad2deg(qi[2]), np.rad2deg(qf[2]), np.rad2deg(dq[2]), e_elbow))
    print('qi({:.4f}) \tqf({:.4f}) \t dq({:.4f}) \t\t e4({:.4f})'.format(np.rad2deg(qi[3]), np.rad2deg(qf[3]), np.rad2deg(dq[3]), e_wrist_pitch))
    print('qi({:.4f}) \tqf({:.4f}) \t dq({:.4f}) \t\t e5({:.4f})'.format(np.rad2deg(qi[4]), np.rad2deg(qf[4]), np.rad2deg(dq[4]), e_wrist_roll))
    print('**************************************************************************')
    
    
    motorPos(e_base, 1)
    
    # PULSO
    if roll != 0:
        a = send_command(roll, '+roll')
        if a == 0:
            print('Erro (roll) - Nao foi possivel movimentar o pulso')
        
    if qi[2]<qf[2]:
        motorPos(e_shoulder, 2)
        motorPos(e_elbow, 3)
    else:
        motorPos(e_elbow, 3)
        motorPos(e_shoulder, 2)
    
    # PULSO
    a = 0
    if dq[3] < 0 and dq[3]!=0:
        a = send_command(np.rad2deg(dq[3]), 'down')
    elif  dq[3] > 0 and dq[3]!=0:
        a = send_command(np.rad2deg(dq[3]), 'up')
    
    if a == 0:
        print('Erro (pitch) - Nao foi possivel movimentar o pulso')
        

        
    if gripper == 'catch':
        motorPos(1200, 4)
    elif gripper == 'release':
        motorPos(-1200, 4)
    else:
        pass
        
def homePosition(roll, q):
    
    
    homeP = np.array([np.deg2rad(180), np.deg2rad(-80), np.deg2rad(75), np.deg2rad(0), np.deg2rad(0)])
    
    d = homeP - q
    
    
    
    btn_base = "P9_21"     # GPS UART2 TX
    btn_shoulder = "P9_29" # SPI1 MISO
    btn_elbow = "P9_28"    # BLUE_GP0_PIN_6 (3_17)
    
    gpio.setup(btn_base, gpio.IN)
    gpio.setup(btn_shoulder, gpio.IN)
    gpio.setup(btn_elbow, gpio.IN)
    

    rcpy.set_state(rcpy.RUNNING)
    
        
    while gpio.input(btn_shoulder):
        while gpio.input(btn_elbow):
            motor.set(3, 1) 
            time.sleep(.3)
            motor.set_brake(3)
            motor.set_free_spin(3)
        motor.set(3, 1)
        motor.set(2, 1) 
        time.sleep(.1)
        motor.set_brake(2)
        motor.set_free_spin(2)
        motor.set_brake(3)
        motor.set_free_spin(3)
    
    v = -1 if d[2] < 0 else 1

    while not(gpio.input(btn_elbow)):
        motor.set(3, v) 
        time.sleep(0.1)
        motor.set_brake(3)
        motor.set_free_spin(3)  
        
    v = -1 if d[0] < np.deg2rad(180) else 1
   
    while not(gpio.input(btn_base)):
        motor.set(1, v) 
        time.sleep(.1)
        motor.set_brake(1)
        motor.set_free_spin(1)

    rcpy.exit()
    
    # PULSO
    a = send_command(roll, '-roll')
    
    if a == 0:
        print('Erro (pitch) - Nao foi possivel movimentar o pulso')
        