#!/usr/bin/env python3
import sys


def help():
    print("Usage: %s <dest_file> <port_string> [local_base_port]" % sys.argv[0])
    sys.exit(1)

def main():
    if len(sys.argv) < 3:
        help()
    dest_file = sys.argv[1]
    hostname = dest_file.split('/')[-1]
    port_string = sys.argv[2]
    local_base_port = sys.argv[3] if len(sys.argv) > 3 else None
    remote_base_port = sys.argv[4] if len(sys.argv) > 3 else None
    PORT_FH = open(dest_file, 'w')
    PORT_FH.write("Host %s\n" % hostname)
    ports = filter(lambda x: str.isdigit(x.split(':')[0]), port_string.split(','))
    for idx, port in enumerate(ports):
        port, name = port.split(':')
        local_port = str(int(local_base_port) + (int(port) - int(remote_base_port))) if local_base_port is not None else port
        PORT_FH.write("\t# %s\n" % name)
        PORT_FH.write("\tLocalForward %s localhost:%s\n" % (local_port, port))
    PORT_FH.close()

if __name__ == '__main__':
    main()
