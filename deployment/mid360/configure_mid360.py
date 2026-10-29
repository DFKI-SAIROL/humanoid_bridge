#!/usr/bin/env python3
"""Write driver IP configuration only; never modify interface or radar network settings."""
import argparse
import ipaddress
import json
from pathlib import Path
import subprocess


def make_config(lidar_ip, host_ip):
    lidar, host = ipaddress.IPv4Address(lidar_ip), ipaddress.IPv4Address(host_ip)
    if lidar == host or any(x.is_multicast or x.is_unspecified or x.is_loopback for x in (lidar, host)):
        raise ValueError('雷达与接收主机必须使用不同的有效单播 IPv4 地址')
    ports = {'cmd_data': 56100, 'push_msg': 56200, 'point_data': 56300, 'imu_data': 56400, 'log_data': 56500}
    host_info = {}
    for name, port in ports.items():
        host_info[name + '_ip'] = '' if name == 'log_data' else str(host)
        host_info[name + '_port'] = port + 1
    return {'lidar_summary_info': {'lidar_type': 8},
            'MID360': {'lidar_net_info': {k + '_port': v for k, v in ports.items()}, 'host_net_info': host_info},
            'lidar_configs': [{'ip': str(lidar), 'pcl_data_type': 1, 'pattern_mode': 0,
                              'extrinsic_parameter': {'roll': 0., 'pitch': 0., 'yaw': 0., 'x': 0, 'y': 0, 'z': 0}}]}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--lidar-ip', required=True)
    p.add_argument('--host-ip', required=True)
    p.add_argument('--force', action='store_true', help='Explicitly replace an existing generated config')
    args = p.parse_args()
    try:
        config = make_config(args.lidar_ip, args.host_ip)
        interfaces = json.loads(subprocess.check_output(['ip', '-j', '-4', 'addr'], text=True))
        if not any(a.get('local') == args.host_ip for interface in interfaces for a in interface.get('addr_info', [])):
            raise ValueError(f'{args.host_ip} 未分配给本机网口；本脚本不会替你修改网络')
        destination = Path(__file__).resolve().parents[1] / 'config/MID360.json'
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.exists() and not args.force:
            raise ValueError('已有配置，未覆盖；确认后可加 --force')
        destination.write_text(json.dumps(config, indent=2) + '\n')
        print(f'已写驱动配置：{destination}；未改变网络，未启动雷达')
    except (ValueError, OSError, subprocess.SubprocessError) as exc:
        p.error(str(exc))


if __name__ == '__main__':
    main()
