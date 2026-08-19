#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
独立验证相机原始数据源的帧间隔均匀性。
直接订阅 4 路 image_raw（不经 infer_nodelet），统计每路相邻帧间隔：
  - hdr_gap : 相机 header.stamp 间隔（打戳时间）
  - recv_gap: 本地接收 wall-clock 间隔（真实到达时间）
对比两者判断：数据源打戳是否均匀、到达是否均匀。
"""
import rospy
import sys
import collections
from sensor_msgs.msg import Image

DIRS = ["upstream", "downstream"]
FOCALS = ["long", "short"]
POLE = "k3_600"

class CamProbe:
    def __init__(self, name, topic):
        self.name = name
        self.topic = topic
        self.prev_hdr_ms = None
        self.prev_recv_ms = None
        self.first = True
        self.hdr_gaps = []
        self.recv_gaps = []
        self.cnt = 0
        self.sub = rospy.Subscriber(topic, Image, self.cb, queue_size=1)

    def cb(self, msg):
        now = rospy.Time.now()
        recv_ms = now.secs * 1000 + now.nsecs // 1000000
        hdr_ms = msg.header.stamp.secs * 1000 + msg.header.stamp.nsecs // 1000000
        self.cnt += 1
        if self.first:
            self.first = False
        else:
            hdr_gap = hdr_ms - self.prev_hdr_ms
            recv_gap = recv_ms - self.prev_recv_ms
            self.hdr_gaps.append(hdr_gap)
            self.recv_gaps.append(recv_gap)
            rospy.loginfo("[%s] hdr_gap=%dms recv_gap=%dms", self.name, hdr_gap, recv_gap)
        self.prev_hdr_ms = hdr_ms
        self.prev_recv_ms = recv_ms

    def report(self):
        def stats(gaps):
            if not gaps:
                return "n=0"
            g = sorted(gaps)
            n = len(g)
            avg = sum(g) / float(n)
            # 分布计数（按 10ms 桶）
            bucket = collections.Counter(int(x // 10) * 10 for x in g)
            dist = dict(sorted(bucket.items()))
            return ("n=%d min=%dms max=%dms avg=%.1fms "
                    "dist={%s}") % (n, g[0], g[-1], avg,
                                    ", ".join("%d:%d" % (k, v) for k, v in dist.items()))
        rospy.loginfo("==== %s  [%s] ====", self.name, self.topic)
        rospy.loginfo("  header.stamp 间隔: %s", stats(self.hdr_gaps))
        rospy.loginfo("  本地到达 间隔: %s", stats(self.recv_gaps))

def main():
    rospy.init_node("cam_fps_probe", anonymous=True)
    probes = []
    for d in DIRS:
        for f in FOCALS:
            topic = "/%s/%s/%s_camera/image_raw" % (POLE, d, f)
            name = "%s_%s" % (d, f)
            p = CamProbe(name, topic)
            probes.append(p)
            rospy.loginfo("Subscribing %s <- %s", name, topic)
    dur = 15
    rospy.loginfo("Collecting for %d seconds...", dur)
    rospy.sleep(dur)
    for p in probes:
        p.report()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
