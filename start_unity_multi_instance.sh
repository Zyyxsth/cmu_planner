#!/bin/bash
# Start multiple Unity instances using network namespaces for port isolation
# This allows each Unity instance to use the same internal port 10000
# but connects to different host ports via port forwarding

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
UNITY_BASE_DIR="$SCRIPT_DIR/src/base_autonomy/vehicle_simulator/mesh/unity/environment"
ROBOT_NUM=${1:-2}

# Check if running as root (required for network namespaces)
if [ "$EUID" -ne 0 ]; then 
    echo "Warning: Not running as root. Network namespaces may not work."
    echo "Try: sudo $0 $ROBOT_NUM"
fi

echo "Starting $ROBOT_NUM Unity instances with port isolation..."

# Cleanup function
cleanup() {
    echo "Cleaning up..."
    for ((i=0; i<ROBOT_NUM; i++)); do
        ip netns del unity_ns_$i 2>/dev/null || true
        killall -9 "Model.x86_64" 2>/dev/null || true
    done
}
trap cleanup EXIT

for ((i=0; i<ROBOT_NUM; i++)); do
    PORT=$((10000 + i))
    NS_NAME="unity_ns_$i"
    
    echo "Setting up Unity instance $i (port $PORT)..."
    
    # Create network namespace
    ip netns add $NS_NAME 2>/dev/null || true
    
    # Set up loopback in namespace
    ip netns exec $NS_NAME ip link set lo up 2>/dev/null || true
    
    # Create veth pair for communication
    ip link del veth_host_$i 2>/dev/null || true
    ip link add veth_host_$i type veth peer name veth_ns_$i
    ip link set veth_ns_$i netns $NS_NAME
    ip addr add 10.200.$i.1/24 dev veth_host_$i
    ip netns exec $NS_NAME ip addr add 10.200.$i.2/24 dev veth_ns_$i
    ip link set veth_host_$i up
    ip netns exec $NS_NAME ip link set veth_ns_$i up
    
    # Set up port forwarding: host:$PORT -> namespace:10000
    ip netns exec $NS_NAME iptables -t nat -A PREROUTING -p tcp --dport 10000 -j REDIRECT --to-port 10000 2>/dev/null || true
    iptables -t nat -A PREROUTING -p tcp --dport $PORT -j DNAT --to-destination 10.200.$i.2:10000 2>/dev/null || true
    iptables -t nat -A POSTROUTING -j MASQUERADE 2>/dev/null || true
    
    # Enable IP forwarding
    echo 1 > /proc/sys/net/ipv4/ip_forward 2>/dev/null || true
    
    # Start Unity in network namespace
    echo "Starting Unity instance $i in namespace $NS_NAME..."
    ip netns exec $NS_NAME "$UNITY_BASE_DIR/Model.x86_64" &
    
    sleep 3
done

echo ""
echo "==================================="
echo "$ROBOT_NUM Unity instances started:"
for ((i=0; i<ROBOT_NUM; i++)); do
    echo "  Instance $i: connect to port $((10000 + i))"
done
echo "==================================="
echo ""
echo "Press Ctrl+C to stop all instances"
wait
