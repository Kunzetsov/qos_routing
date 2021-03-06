import sys

from ryu.cmd import manager


def main():
    sys.argv.append('--ofp-tcp-listen-port')
    sys.argv.append('6653')
    sys.argv.append('--observe-links')
    # sys.argv.append('ryu/ryu/app/gui_topology/gui_topology.py')
    sys.argv.append('qos_route_app')
    # sys.argv.append('ofctl_rest')

    # sys.argv.append('test_controller')
    # sys.argv.append('--verbose')
    sys.argv.append('--enable-debugger')
    manager.main()


if __name__ == '__main__':
    main()
