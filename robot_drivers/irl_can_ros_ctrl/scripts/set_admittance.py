#!/usr/bin/env python2
'''
This script set the admittance of the arm motors
of jn0 by calling the set_admittance services.
Usage: set_admittance.py [--side {left,right}] [--values 0 0 0 0]
Where values are the admittance values of 
    [elbow_tilt_motor, shoulder_pan_motor, shoulder_roll_motor, shoulder_tilt_motor]
'''
import argparse
import rospy
from irl_can_ros_ctrl.srv import SetAdmittance

def get_args():
    parser = argparse.ArgumentParser(
        description="Wait and publish to jn0's set_admittance services"
    )
    parser.add_argument("--side", "-s", default="left", choices=["left", "right"])
    parser.add_argument("--values", "-v", type=int, nargs=4, default=[0, 0, 0, 0])

    return parser.parse_known_args()

def main():
    (args, _) = get_args()

    namespace = "jn0"
    package = "jn0_driver"
    service_name = "set_admittance"
    motors = [
        "{side}_elbow_tilt_motor",
        "{side}_shoulder_pan_motor",
        "{side}_shoulder_roll_motor",
        "{side}_shoulder_tilt_motor"
    ]
    sides = {
        'left': 'L',
        'right': 'R'
    }
    side = sides[args.side]

    services = map(lambda motor: \
        "/{0}/{1}/{2}/{3}" \
        .format(namespace, package, motor.format(side=side), service_name), \
        motors)

    # Wait for the services
    try:
        for service in services:
            rospy.wait_for_service(service, timeout=10)
    except rospy.ROSException:
        print('Services timed out after 10 sec.')
        exit(1)

    # Create the proxies
    service_proxies = map(lambda service: rospy.ServiceProxy(service, SetAdmittance), \
                        services)

    # Set the admittance for each motors
    for i, service_proxy in enumerate(service_proxies):
        print("Sending to service '{0}' m=0, b=0, k={1}".format(services[i], args.values[i]))
        service_proxy(0, 0, args.values[i])

if __name__ == "__main__":
    main()
