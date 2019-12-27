from optparse import OptionParser
import socket
import logging

# sudo python udp_proxy.py --bind-address 0.0.0.0 --port=7777 --dst-ip=192.168.10.1 --dst-port=8999 --dst-network-card=wlp5s0 --duplex=1

# sudo python udp_proxy.py --bind-address 127.0.0.1 --port=11111 --dst-ip=127.0.0.1 --dst-port=22222 --src-network-card=wlp5s0 --duplex=0

# sudo python udp_proxy.py --bind-address 0.0.0.0 --port=8999 --dst-ip=127.0.0.1 --dst-port=7777 --src-network-card=wlp5s0 --duplex=0 


logger = logging.getLogger()
handler = logging.StreamHandler()
handler.setLevel(logging.DEBUG)
formatter = logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")
handler.setFormatter(formatter)
logger.setLevel(logging.DEBUG)
logger.addHandler(handler)


def parse_args():
    parser = OptionParser()

    parser.add_option('--bind-address',
                      help='The address to bind, use 0.0.0.0 for all ip address.')
    parser.add_option('--port',
                      help='The port to listen, eg. 623.',
                      type=int)
    parser.add_option('--dst-ip',
                      help='Destination host ip, eg. 192.168.3.101.')
    parser.add_option('--dst-port',
                      help='Destination host port, eg. 623.',
                      type=int)
    parser.add_option('--src-network-card',
                      help='Source Network Card, eg. wlp5s0, 0 if none')

    parser.add_option('--dst-network-card',
                      help='Dest Network Card, eg. wlp5s0, 0 if none')

    parser.add_option('--duplex', help='Duplex Mode, eg 1', type=int)

    return parser.parse_args()

(options, args) = parse_args()
print(options, args)


def recv():
    sock_src = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_dst = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # sock_src.setsockopt(socket)
    sock_src.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock_dst.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    if not options.src_network_card == None:
        print('Using {} for source socket'.format(options.src_network_card))
        sock_src.setsockopt(socket.SOL_SOCKET, 25, options.src_network_card)

    if not options.dst_network_card == None:
        print('Using {} for dest socket'.format(options.dst_network_card))
        sock_dst.setsockopt(socket.SOL_SOCKET, 25, options.dst_network_card)

    recv_addr = (options.bind_address, options.port)
    dst_addr = (options.dst_ip, options.dst_port)
    print("dst_addr={}".format(dst_addr))
    print("recv_addr={}".format(recv_addr))
    sock_src.bind(recv_addr)

    while True:
        data, addr = sock_src.recvfrom(65565)
        if not data:
            logger.error('an error occured')
            break
        logger.debug('received: {0!r} from {1}'.format(data, addr))
        sock_dst.sendto(data, dst_addr)
        if options.duplex:
            data, _  = sock_dst.recvfrom(65565)
            sock_src.sendto(data, addr)

    sock_src.close()
    sock_dst.close()


if __name__ == '__main__':
    parse_args()
    try:
        recv()
    except KeyboardInterrupt:
        exit(0)