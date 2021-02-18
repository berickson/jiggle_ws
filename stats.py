#!/usr/bin/env python
import rospy
from car_msgs.msg import speedometer
from std_msgs.msg import Float32
import numpy as np

stats = {}
last_values = {}
pubs = {}

def callback(name, msg):
    global stats, last_values

    if not name in stats:
        stats[name] = []
        last_values[name] = 0

    last_v = last_values[name]
    v_array = stats[name]

    
    v_array.append(msg.v-last_v)
    last_v = msg.v
    if(len(v_array)>=1000):
        rospy.loginfo("name: %s std: %.2f avg: %.2f",name, np.std(v_array), np.mean(v_array))
        pubs[name].publish(np.std(v_array))
        del stats[name][:]

    last_values[name] = msg.v


  
    
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('stats_listener', anonymous=True)

    rospy.Subscriber("/car/speedometers/fr", speedometer, lambda msg : callback("fr",msg ) )
    rospy.Subscriber("/car/speedometers/fl", speedometer, lambda msg : callback("fl",msg ) )
    rospy.Subscriber("/car/speedometers/motor", speedometer, lambda msg : callback("motor",msg ) )

    pubs["fr"] = rospy.Publisher('std/fr', Float32, queue_size=10)
    pubs["fl"] = rospy.Publisher('std/fl', Float32, queue_size=10)
    pubs["motor"] = rospy.Publisher('std/motor', Float32, queue_size=10)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()