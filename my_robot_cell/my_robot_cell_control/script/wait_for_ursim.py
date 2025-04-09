#!/usr/bin/env python3

import socket
import sys
import time


def wait_for_port(ip, port, timeout=60.0):
    start_time = time.time()
    while time.time() - start_time < timeout:
        try:
            with socket.create_connection((ip, port), timeout=2):
                print(f"✅ URSim is up on {ip}:{port}")
                return True
        except (socket.timeout, ConnectionRefusedError, OSError):
            print(f"⏳ Waiting for URSim on {ip}:{port}...")
            time.sleep(2)
    print("❌ Timeout waiting for URSim")
    return False

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: wait_for_ursim.py <ip> <port>")
        sys.exit(1)

    ip = sys.argv[1]
    port = int(sys.argv[2])

    if wait_for_port(ip, port):
        sys.exit(0)
    else:
        sys.exit(1)
