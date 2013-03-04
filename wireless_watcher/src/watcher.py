#!/usr/bin/python

import roslib; roslib.load_manifest('wireless_watcher')

import rospy
import wireless_msgs.msg
import subprocess, re


rospy.init_node('wireless_watcher')


hz = rospy.get_param('~hz', 1)
dev = rospy.get_param('~dev', 'wlan0')
previous_error = False
previous_success = False
r = None

def trim(field):
    m = re.match('[0-9-.]+', field)
    if m:
        return int(m.group(0))

class Connection(wireless_msgs.msg.Connection):
    def __init__(self, fields):
        args = {}
        args['bitrate'] = trim(fields['Bit Rate'])
        args['txpower'] = trim(fields['Tx-Power'])
        args['link_quality_raw'] = fields['Link Quality']
        num, den = re.split('/', args['link_quality_raw']) 
        args['link_quality'] = float(num) / float(den)
        args['signal_level'] = trim(fields['Signal level'])
        # args['noise_level'] = trim(fields['Noise level'])
        super(Connection, self).__init__(**args)

class Network(wireless_msgs.msg.Network):
    def __init__(self, fields):
        args = {}
        super(Network, self).__init__(**args)


connection_pub = rospy.Publisher('connection', Connection)
network_pub = rospy.Publisher('network', Network)


while not rospy.is_shutdown():
  try:
    wifi_str = subprocess.check_output(['iwconfig', dev], stderr=subprocess.STDOUT);
    fields_str = re.split('\s\s+', wifi_str)
    fields_list = [re.split('[:=]', field_str, maxsplit=1) for field_str in fields_str]
    fields_dict = { 'dev': fields_str[0], 'type': fields_str[1] }
    fields_dict.update(dict([field for field in fields_list if len(field) == 2 ]))
    #print fields_dict
    connection_msg = Connection(fields_dict)
    connection_pub.publish(connection_msg)

    if not previous_success:
      previous_success = True
      previous_error = False
      rospy.loginfo("Retrieved status of interface %s. Now updating at %f Hz." % (dev, hz))
      r = rospy.Rate(hz)

  except subprocess.CalledProcessError:
    if not previous_error:
      previous_error = True
      previous_success = False
      rospy.logerr("Error checking status of interface %s. Will try again every 10s." % dev)
      r = rospy.Rate(0.1)
    
  r.sleep()
