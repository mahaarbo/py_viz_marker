#!/usr/bin/env python

import rospy
import visualization_msgs.msg as viz_msg
import py_viz_marker.srv as srvs
import threading

# Using labels this way instead of IDs is not optimal
# but it works for my purposes.


class VisualizationHandler(object):
    def __init__(self, namespace="", rate=50):
        rospy.init_node("vizhandler", anonymous=True)
        self.namespace = namespace
        self.lifetime = rospy.Time.from_sec(1.0/rate)
        self.rate = rospy.Rate(rate)
        self.lock = threading.Lock()
        self.markers = {}

        self.services = [
            rospy.Service(namespace + "/add_marker",
                          srvs.AddMarker,
                          self.add_marker),
            rospy.Service(namespace + "/remove_marker",
                          srvs.RemoveMarker,
                          self.remove_marker)
        ]
        self.publisher = rospy.Publisher("visualization_marker_array",
                                         viz_msg.MarkerArray,
                                         queue_size=1)

    def run(self):
        while not rospy.is_shutdown():
            self.publish()
            self.rate.sleep()

    def publish(self):
        mrks = []
        mrk_idx = 0
        with self.lock:
            for mrk_lbl, mrk_msg in self.markers.items():
                mrk_msg.header.stamp = rospy.Time.now()
                mrk_msg.id = mrk_idx
                mrks.append(mrk_msg)
                mrk_idx += 1
            self.publisher.publish(mrks)

    def add_marker(self, req):
        with self.lock:
            req.marker.lifetime = self.lifetime
            self.markers[req.label] = req.marker
        return srvs.AddMarkerResponse()

    def remove_marker(self, req):
        with self.lock:
            del self.markers[req.label]
        return srvs.RemoveMarkerResponse()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Node to publish "
                                     + "viz_markers, uses services.")
    parser.add_argument("-namespace",
                        help="Namespace to publish in.",
                        type=str,
                        default="")
    parser.add_argument("-rate",
                        help="Update rate and inverse lifetime.",
                        type=int,
                        default=50)
    args, unknown_args = parser.parse_known_args()
    viz_handler = VisualizationHandler(args.namespace, args.rate)
    try:
        viz_handler.run()
    except rospy.ROSInterruptException:
        pass
