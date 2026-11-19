# Source this in both the bridge and policy terminals:
# source /path/to/humanoid_bridge/setup_x2_wifi.sh <robot-wifi-interface>
# ROS_DOMAIN_ID defaults to 0; set it beforehand if the robot uses another domain.
# Routes the motion computer through the hotspot using a temporary NetworkManager
# device setting. The saved Wi-Fi profile and default route are not changed.

_setup_x2_wifi() {
    local interface="${1:-}"
    local wifi_ip profile script_dir route attempt
    local robot_ip=10.0.1.40
    local gateway=192.168.88.88
    if [[ -z "$interface" ]]; then
        echo "Usage: source setup_x2_wifi.sh <robot-wifi-interface>" >&2
        return 1
    fi

    wifi_ip="$(ip -4 -o addr show dev "$interface" scope global | awk 'NR == 1 {split($4, a, "/"); print a[1]}')"
    if [[ -z "$wifi_ip" ]]; then
        echo "No IPv4 address on $interface; connect it to the X2 hotspot first." >&2
        return 1
    fi

    script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    source "$script_dir/setup_x2.sh" || return 1

    # The robot's DDS endpoints use its internal address. An interface whitelist
    # alone neither routes that address over Wi-Fi nor discovers it by unicast.
    route="$(ip -4 route get "$robot_ip" 2>/dev/null)"
    if [[ " $route " != *" dev $interface "* ]]; then
        if ! nmcli device modify "$interface" +ipv4.routes "$robot_ip/32 $gateway"; then
            echo "Could not route $robot_ip through the X2 hotspot on $interface." >&2
            return 1
        fi
        # NetworkManager may return before the kernel route is visible.
        for attempt in {1..20}; do
            route="$(ip -4 route get "$robot_ip" 2>/dev/null)"
            [[ " $route " == *" dev $interface "* ]] && break
            sleep 0.1
        done
        if [[ " $route " != *" dev $interface "* ]]; then
            echo "Route to $robot_ip still does not use $interface; check ip route." >&2
            return 1
        fi
    fi

    profile="$(mktemp /tmp/x2-wifi-dds.XXXXXX.xml)" || return 1
    if ! cat > "$profile" <<EOF
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>x2_wifi</transport_id>
      <type>UDPv4</type>
      <maxInitialPeersRange>64</maxInitialPeersRange>
      <interfaceWhiteList>
        <address>$wifi_ip</address>
        <address>127.0.0.1</address>
      </interfaceWhiteList>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="x2_wifi" is_default_profile="true">
    <rtps>
      <builtin>
        <initialPeersList>
          <!-- Explicit peers must also discover the local bridge, policy and joystick. -->
          <locator><udpv4><address>127.0.0.1</address></udpv4></locator>
          <locator><udpv4><address>$robot_ip</address></udpv4></locator>
        </initialPeersList>
      </builtin>
      <userTransports><transport_id>x2_wifi</transport_id></userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
EOF
    then
        rm -f "$profile"
        return 1
    fi

    unset ROS_DISCOVERY_SERVER FASTDDS_DEFAULT_PROFILES_FILE RMW_FASTRTPS_USE_QOS_FROM_XML
    export FASTRTPS_DEFAULT_PROFILES_FILE="$profile"
    export ROS_LOCALHOST_ONLY=0
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
    echo "X2 DDS: $interface ($wifi_ip) + loopback only; domain $ROS_DOMAIN_ID"
    echo "Robot DDS peer: $robot_ip via hotspot $gateway"
    echo "DDS profile: $profile (source again and restart nodes if the Wi-Fi address changes)"
}

if ! _setup_x2_wifi "$@"; then
    unset -f _setup_x2_wifi
    return 1
fi
unset -f _setup_x2_wifi
