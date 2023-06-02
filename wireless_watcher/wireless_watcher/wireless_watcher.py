#!/usr/bin/env python3
# Software License Agreement (BSD)
#
# @author    Mike Purvis <mpurvis@clearpath.ai>
# @author    Roni Kreinin <rkreinin@clearpathrobotics.com>
# @copyright (c) 2022, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this list of conditions and the
#   following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
#   following disclaimer in the documentation and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or
#   promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import re
import rclpy
import time
import threading
import std_msgs.msg
import subprocess

from wireless_msgs.msg import Connection, Network

from rclpy.qos import qos_profile_sensor_data

SYS_NET_PATH = '/sys/class/net'


def trim(field, type=int):
    m = re.match('[0-9-.]+', field)
    if m:
        return type(m.group(0))


class WirelessConnection(Connection):
    def __init__(self, fields):
        args = {}

        args['bitrate'] = trim(fields.get('Bit Rate', "0.0"), float)
        args['txpower'] = trim(fields.get('Tx-Power', "0"))
        args['signal_level'] = trim(fields.get('Signal level', "0"))
        args['noise_level'] = trim(fields.get('Noise level', "0"))
        args['essid'] = fields.get('ESSID', "").strip('"')
        args['bssid'] = fields.get('Access Point', "").strip()
        args['frequency'] = trim(fields.get('Frequency', "0.0"), float)

        try:
            args['link_quality_raw'] = fields['Link Quality']
            num, den = re.split('/', args['link_quality_raw'])
            args['link_quality'] = float(num) / float(den)
        except:
            pass

        self.msg = Connection(**args)


class WirelessNetwork(Network):
    def __init__(self, fields):
        args = {}
        self.msg = Network(**args)


def main():
    rclpy.init()

    node = rclpy.create_node('wireless_watcher')

    node.declare_parameter('hz', 1)
    node.declare_parameter('dev', '')
    node.declare_parameter('connected_topic', 'connected')
    node.declare_parameter('connection_topic', 'connection')

    hz = node.get_parameter('hz').value
    dev = node.get_parameter('dev').value
    connected_topic = node.get_parameter('connected_topic').value
    connection_topic = node.get_parameter('connection_topic').value

    if not dev:
        wldevs = [d for d in os.listdir(SYS_NET_PATH) if d.startswith('wl') or d.startswith('wifi')]
        if wldevs:
            dev = wldevs[0]
        else:
            node.get_logger().fatal('No wireless device found to monitor.')
            return 1

    node.get_logger().info('Monitoring {0}'.format(dev))

    previous_error = False
    previous_success = False

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    connected_pub = node.create_publisher(std_msgs.msg.Bool, connected_topic, qos_profile_sensor_data)
    connection_pub = node.create_publisher(Connection, connection_topic, qos_profile_sensor_data)

    # Disable this until we actually collect and publish the data.
    # network_pub = node.create_publisher(Network, 'network', qos_profile_sensor_data)

    connected_msg = std_msgs.msg.Bool()

    while rclpy.ok():
        try:
            with open(os.path.join(SYS_NET_PATH, dev, 'operstate'), 'rb') as f:
                connected_msg.data = f.read().strip() == b'up'
        except FileNotFoundError:
            connected_msg.data = False
        finally:
            connected_pub.publish(connected_msg)

        try:
            wifi_str = subprocess.check_output(['iwconfig', dev], stderr=subprocess.STDOUT).decode()
            fields_str = re.split('\s\s+', wifi_str)
            fields_list = [re.split('[:=]', field_str, maxsplit=1) for field_str in fields_str]
            fields_dict = {'dev': fields_str[0], 'type': fields_str[1]}
            fields_dict.update(dict([field for field in fields_list if len(field) == 2]))

            # Check if WiFi is connected to a network
            if "Not-Associated" not in fields_dict['Access Point']:
                connection = WirelessConnection(fields_dict)
                connection_pub.publish(connection.msg)

            if not previous_success:
                previous_success = True
                previous_error = False
                node.get_logger().info('Retrieved status of interface {0}. Now updating at {1} Hz.'.format(dev, hz))

        except subprocess.CalledProcessError:
            if not previous_error:
                previous_error = True
                previous_success = False
                node.get_logger().error('Error checking status of interface {0}. Will try again at {1} Hz.'.format(dev, hz))

        time.sleep(1.0 / hz)

    thread.join()
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
