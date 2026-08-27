#!/bin/bash
# Launch ArduPlane SITL against RealFlight Evolution on the Windows host.
# WSL2 is NAT'd, so the host address is the default gateway and changes on reboot.
set -e

HOST_IP=$(ip route show default | awk '{print $3; exit}')
if [ -z "$HOST_IP" ]; then
    echo "could not determine Windows host IP from default route" >&2
    exit 1
fi

echo "Windows host: $HOST_IP"
if ! timeout 3 bash -c "cat < /dev/null > /dev/tcp/$HOST_IP/18083" 2>/dev/null; then
    echo "WARNING: $HOST_IP:18083 not reachable."
    echo "  - is RealFlight running with 'RealFlight Link Enabled' ticked?"
    echo "  - does the Windows firewall allow inbound TCP 18083 from this subnet?"
fi

exec sim_vehicle.py -v ArduPlane -f "flightaxis:$HOST_IP" \
    --console --map "$@"
