"""Temporarily set a GigE Vision camera's IP (GVCP FORCEIP, lost on camera power-cycle).
Usage: python gige_forceip.py <MAC aa:bb:..> <ip> [mask] [gateway]"""
import socket
import struct
import sys

mac = bytes.fromhex(sys.argv[1].replace(":", "").replace("-", ""))
ip, mask = sys.argv[2], (sys.argv[3] if len(sys.argv) > 3 else "255.255.255.0")
gw = sys.argv[4] if len(sys.argv) > 4 else "0.0.0.0"
payload = (b"\0\0" + mac + b"\0" * 12 + socket.inet_aton(ip) + b"\0" * 12
           + socket.inet_aton(mask) + b"\0" * 12 + socket.inet_aton(gw))
pkt = struct.pack(">BBHHH", 0x42, 0x11, 0x0004, len(payload), 2) + payload
sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sk.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sk.bind(("172.20.5.10", 0))
sk.settimeout(2)
sk.sendto(pkt, ("255.255.255.255", 3956))
try:
    data, src = sk.recvfrom(1024)
    print("FORCEIP ack from", src, "status", hex(struct.unpack(">H", data[:2])[0]))
except socket.timeout:
    print("no ack")
