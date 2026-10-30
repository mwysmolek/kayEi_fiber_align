"""GigE Vision (GVCP) discovery: broadcast DISCOVERY_CMD on every local IPv4 interface."""
import socket
import struct
import subprocess
import re
import time

GVCP_PORT = 3956


def local_ips():
    out = subprocess.run(["ipconfig"], capture_output=True, text=True).stdout
    return [ip for ip in re.findall(r"IPv4[^:]*:\s*([\d.]+)", out) if not ip.startswith("127.")]


def parse(data):
    p = data[8:]
    s = lambda a, b: p[a:b].split(b"\0")[0].decode(errors="replace").strip()
    mac = ":".join(f"{x:02X}" for x in p[10:16])
    ip = socket.inet_ntoa(p[36:40])
    mask = socket.inet_ntoa(p[52:56])
    gw = socket.inet_ntoa(p[68:72])
    cfg_opts, cfg_cur = struct.unpack(">II", p[16:24])
    modes = {1: "persistent", 2: "DHCP", 4: "LLA"}
    cur = [n for b, n in modes.items() if cfg_cur & b]
    return dict(ip=ip, mask=mask, gw=gw, mac=mac, vendor=s(72, 104), model=s(104, 136),
                version=s(136, 168), serial=s(216, 232), name=s(232, 248), ipmode="+".join(cur))


def discover(timeout=1.5):
    found = {}
    for lip in local_ips():
        sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sk.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            sk.bind((lip, 0))
        except OSError:
            continue
        # 0x42 key, flags 0x11 = ack required + allow broadcast ack, cmd 0x0002, len 0, req id 1
        sk.sendto(struct.pack(">BBHHH", 0x42, 0x11, 0x0002, 0, 1), ("255.255.255.255", GVCP_PORT))
        sk.settimeout(0.3)
        end = time.time() + timeout
        while time.time() < end:
            try:
                data, src = sk.recvfrom(1024)
            except socket.timeout:
                continue
            if len(data) >= 256 and data[2:4] == b"\x00\x03":
                d = parse(data)
                d["via"] = lip
                found[d["mac"]] = d
        sk.close()
    return list(found.values())


if __name__ == "__main__":
    cams = discover()
    if not cams:
        print("no GigE Vision devices found")
    for c in cams:
        for k, v in c.items():
            print(f"{k:8}: {v}")
        print()
