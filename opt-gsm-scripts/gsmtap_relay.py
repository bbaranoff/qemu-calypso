#!/usr/bin/env python3
# Pont GSMTAP 4729 -> 4730 : grgsm_decode envoie sur 127.0.0.1:4729 (hardcode),
# le shunt ecoute sur 4730. On rebalance. (4729 host/BTS = 172.20.x.1, libre en loopback.)
import socket
IN=("127.0.0.1",4729); OUT=("127.0.0.1",4730)
r=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
r.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
r.bind(IN)
w=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
print("[gsmtap-relay] 127.0.0.1:4729 -> 127.0.0.1:4730",flush=True)
n=0
while True:
    d,_=r.recvfrom(65536); w.sendto(d,OUT); n+=1
    if n<=5 or n%200==0: print("[gsmtap-relay] %d pkts 4729->4730"%n,flush=True)
