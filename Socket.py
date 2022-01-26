
import socket
import pickle
import Robo as rb
# import Camera as cam
import numpy as np
import copy
import time



def isValid(msg):
    
    tmp = msg.replace(' ', '')
    tmp = msg.split(';')
    
    
    if len(tmp) not in (1, 2, 5):
        print('\nFormato invalido!\n')
        return False
        
    elif tmp[0] == 'q0':
        return True
        
    else:
        try:
            x = float(tmp[0])
            y = float(tmp[1])
            z = float(tmp[2])
            roll = float(tmp[3])
            garra = tmp[4]
            
            
            if x <= -1 or x >= 1:
                print('\nValor invalido da coordenada X\n')
                return False
            if y <= -1 or y >= 1:
                print('\nValor invalido da coordenada Y\n')
                return False
            if z <= -1 or z >= 1:
                print('\nValor invalido da coordenada Z\n')
                return False
                
            if roll < -360 or roll > 360:
                print('\nAngulo de rotacao invalido\n')
                return False
            
            if garra not in ('catch', 'release', 'nothing'):
                print('\nComando invalido para movimento da garra\n')
                return False
                
        except:
            print('\nValores invalidos!\n')
            return False
            
    return True       
 

def ini_pos():
    return np.array([np.deg2rad(180), np.deg2rad(-80), np.deg2rad(75), np.deg2rad(0), np.deg2rad(0)])


def move(qi, p):
    
    
    while p: 
        
        msg = p.pop(0)
        
        if msg == 'q0':
            rb.homePosition(0, qi) 
            return ini_pos()
            
        tmp = msg.replace(' ', '')
        tmp = msg.split(';')
        
        x = float(tmp[0])
        y = float(tmp[1])
        z = float(tmp[2])
        roll = float(tmp[3])
        garra = tmp[4]
        
        p1 = np.array([x, y, z])
        
        q1 = rb.ik(qi, p1)
        
        rb.move(copy.deepcopy(qi), copy.deepcopy(q1), roll, garra)
        
        return q1
        
def connection():
    
    
    HOST = '127.0.0.1' # Endereco IP do Servidor
    PORT = 5020            # Porta que o Servidor esta
    
    tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    orig = (HOST, PORT)
    
    tcp.bind(orig)
    tcp.listen(1)
    
    q0 = ini_pos()
    p = []
    
    while True:
        con, cliente = tcp.accept()
        print ('Concetado por', cliente)
        while True:
            msg = con.recv(1024)
            if not msg: break
            
            msg = pickle.loads(msg)
            
            if isValid(msg):
                print(cliente, msg)
                p.append(msg)
            
            if p:
                q0 = move(q0, p)
            
            
        print ('Finalizando conexao do cliente', cliente)
        con.close()

# connection()

# q0 = ini_pos()
# p = []
# msg = '-.2;-.22;.22;90;nothing'

# if isValid(msg):
#     print(msg)
#     p.append(msg)
#     p.append('q0')

# if p:
#     q0 = move(q0, p)
#     print(np.rad2deg(q0))
#     q0 = move(q0, p)
#     print(np.rad2deg(q0))
