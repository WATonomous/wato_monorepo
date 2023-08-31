import time

import rclpy
from aggregator.aggregator import Aggregator


def test_calc_freq():
    rclpy.init()

    aggregator_node = Aggregator()

    thresh = 100
    start_time = aggregator_node.node_start_time
    test_count = 5

    test_freq = test_count / (time.time() - start_time)

    assert abs(aggregator_node.calc_freq(5) - test_freq) < thresh
