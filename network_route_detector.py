# Copyright (C) 2016 Li Cheng at Beijing University of Posts
# and Telecommunications. www.muzixing.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
# implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import division

import copy

from ryu import cfg
from ryu.base import app_manager
from ryu.base.app_manager import lookup_service_brick
from ryu.controller import ofp_event
from ryu.controller.handler import MAIN_DISPATCHER, DEAD_DISPATCHER
from ryu.controller.handler import set_ev_cls
from ryu.ofproto import ofproto_v1_3
from ryu.lib import hub
from ryu.topology.switches import Switches
from ryu.topology.switches import LLDPPacket
import networkx as nx
import time
import settings

CONF = {
    "weight": 'delay'
}


class NetworkRouteDetector(app_manager.RyuApp):
    """
        NetworkRouteDetector is a Ryu app for collecting link delay.
    """

    OFP_VERSIONS = [ofproto_v1_3.OFP_VERSION]

    def __init__(self, *args, **kwargs):
        super(NetworkRouteDetector, self).__init__(*args, **kwargs)
        self.name = 'network_route_detector'
        self.awareness = lookup_service_brick('awareness')

        self.delay_graph = {}
        self.prev_delay_graph = {}
        self.delay_graph_safe = {}
        self.prev_delay_graph_safe = {}

        self.bandwidth_graph = {}
        self.prev_bandwidth_graph = {}
        self.bandwidth_graph_safe = {}
        self.prev_bandwidth_graph_safe = {}

        self.speed_graph = {}
        self.prev_speed_graph = {}
        self.speed_graph_safe = {}
        self.prev_speed_graph_safe = {}

        self.optimal_criterias_delay_graph = {}
        self.optimal_criterias_speed_graph = {}
        self.optimal_criterias_bandwidth_graph = {}

        self.optimality_graph = {}
        self.prev_optimality_graph = {}
        self.reversed_optimality_graph = {}
        self.prev_reversed_optimality_graph ={}

        self.max_delay = None
        self.max_bandwidth = None
        self.max_speed = None
        self.max_convolution = None

        self.delay_updated = False
        self.bandwidth_updated = False
        self.speed_updated = False
        self.all_stats_updated = False
        self.monitor = hub.spawn(self._route_metrics_monitor)
        self.a1 = 0.3
        self.a2 = -0.3
        self.a3 = -0.3

    def _route_metrics_monitor(self):
        while True:
            # hub.sleep(settings.ROUTE_METRICS_COMPUTING_PERIOD)
            hub.sleep(10)
            self.calculate_metrics()
            hub.sleep(50)

    def handle_bandwidth_update(self, bw_graph):
        # print('handle_bandwidth_update')
        self.bandwidth_updating()
        for src, dst_list in bw_graph.items():
            filtered_dst_list = {dst_node: dst_nodes for dst_node, dst_nodes in dst_list.items() if dst_node != src}
            self.update_bandwidth(src, filtered_dst_list)
            # print(src, dst_list)
        self.bandwidth_graph_safe = copy.copy(self.bandwidth_graph)
        self.prev_bandwidth_graph_safe = copy.copy(self.prev_bandwidth_graph)
        # print(self.bandwidth_graph_safe)
        self.bandwidth_is_updated()
        # print('bandwidth =>> %s' % self.prev_bandwidth_graph)

    def update_bandwidth(self, src, dst_list):
        for dst, bandwidth in dst_list.items():
            self.update_parameter(src, dst, bandwidth, self.bandwidth_graph, self.prev_bandwidth_graph)
    """
        { src : { dst1: delay, dst2: delay} }
    """
    def handle_delay_update(self, delay_graph):
        self.delay_updating()
        for src, dst_list in delay_graph.items():
            filtered_dst_list = {dst_node: dst_nodes for dst_node, dst_nodes in dst_list.items() if dst_node != src}
            self.update_delay(src, filtered_dst_list)

        self.delay_graph_safe = copy.copy(self.delay_graph)
        self.prev_delay_graph_safe = copy.copy(self.prev_delay_graph)
        # self.delay_graph
        # print('handle_delay_update')
        # print(self.bandwidth_graph_safe)
        self.delay_is_updated()
        # print('prev delay =>> %s' % self.prev_delay_graph)
        # print('delay =>> %s' % self.delay_graph)

    def update_delay(self, src, dst_list):
        for dst, delay in dst_list.items():
            self.update_parameter(src, dst, delay, self.delay_graph, self.prev_delay_graph)

    """
        dp_port -- datapath -> port  
    """
    def handle_speed_update(self, dp_port, speed):
        # print('handle_speed_update')
        # (src_dpid, dst_dpid) => (src_port, dst_port)
        self.speed_updating()
        links = {links: ports for links, ports in self.awareness.link_to_port.items()
                 if dp_port[0] in links
                    and dp_port[1] in ports}

        self.update_speed(links, speed)
        self.speed_is_updated()
        # print('prev_speed =>> %s, dp_port =>> %s ' % (self.prev_speed_graph, dp_port))
        # print('speed =>> %s, dp_port =>> %s ' % (self.speed_graph, dp_port))

    def update_speed(self, links, speed):


        for link, ports in links.items():
            (src, dst) = link
            # (src_port, dst_port) = ports
            self.update_parameter(src, dst, speed, self.speed_graph, self.prev_speed_graph)

        self.speed_graph_safe = copy.copy(self.speed_graph)
        self.prev_speed_graph_safe = copy.copy(self.prev_speed_graph)

    def update_parameter(self, src, dst, value, graph, prev_graph):
        if graph.get(src) is None:
            graph.setdefault(src, {})

        if prev_graph.get(src) is None:
            prev_graph.setdefault(src, {})

        if graph.get(src).get(dst) is None:
            graph[src][dst] = 0
            # updated_graph[src][dst] = False

        prev_val = graph.get(src).get(dst)
        prev_graph[src][dst] = prev_val

        if self.need_to_update(prev_val, value):
            graph[src][dst] = value
            # updated_graph[src][dst] = True
            # add watcher here

    def local_convolution(self, z1, z2, z3):
        if z1 is None: z1 = 0
        if z2 is None: z2 = 0
        if z3 is None: z3 = 0
        return self.a1 * z1 + self.a2 * z2 + self.a3 * z3

    def local_optimal_criteria(self, x, x_max):
        if x_max == 0: return 0
        return x / x_max

    def local_reversed_convolution(self, w, w_max):
        return w_max - w + 1

    def calc_optimal_criterias(self):

        self.calc_optimal_criterias_universal(
            self.prev_bandwidth_graph_safe,
            self.bandwidth_graph_safe,
            self.max_bandwidth,
            self.optimal_criterias_bandwidth_graph
        )

        self.calc_optimal_criterias_universal(
            self.prev_delay_graph_safe,
            self.delay_graph_safe,
            self.max_delay,
            self.optimal_criterias_delay_graph
        )

        self.calc_optimal_criterias_universal(
            self.prev_speed_graph_safe,
            self.speed_graph_safe,
            self.max_speed,
            self.optimal_criterias_speed_graph
        )

        # for src, dist_list in self

    def find_max(self, graph):
        max_val = None
        for src, dist_list in graph.items():
            curr_max_link = max(dist_list, key=lambda dst: dist_list[dst])
            curr_max_val = dist_list[curr_max_link]
            if max_val is None or max_val < curr_max_val:
                max_val = curr_max_val
        return max_val

    def calc_optimal_criterias_universal(self, prev_graph, cur_graph, prev_max, optimal_criteria_graph):
        max_val = self.find_max(cur_graph)
        # prev_max = max_val
        for src, dst_list in cur_graph.items():
            if optimal_criteria_graph.get(src) is None:
                optimal_criteria_graph.setdefault(src, {})
            for dst, val in dst_list.items():
                if max_val != prev_max or prev_graph[src][dst] != cur_graph[src][dst]:
                    optimal_criteria_graph[src][dst] = self.local_optimal_criteria(cur_graph[src][dst], max_val)

        prev_max = max_val

    def calc_convolutions(self):
        self.prev_optimality_graph = copy.copy(self.optimality_graph)
        for src, dst_list in self.optimal_criterias_bandwidth_graph.items():
            if self.optimality_graph.get(src) is None:
                self.optimality_graph.setdefault(src, {})
            if self.optimal_criterias_bandwidth_graph.get(src) is None:
                self.optimal_criterias_bandwidth_graph.setdefault(src, {})
            if self.optimal_criterias_delay_graph.get(src) is None:
                self.optimal_criterias_delay_graph.setdefault(src, {})
            if self.optimal_criterias_speed_graph.get(src) is None:
                self.optimal_criterias_speed_graph.setdefault(src, {})

            for dst, val in dst_list.items():
                # print(src, dst, self.optimal_criterias_delay_graph)
                self.optimality_graph[src][dst] = \
                    self.local_convolution(
                        self.optimal_criterias_bandwidth_graph.get(src).get(dst),
                        self.optimal_criterias_delay_graph.get(src).get(dst),
                        self.optimal_criterias_speed_graph.get(src).get(dst)
                    )

    def calc_reversed_convolutions(self):
        max_conv = self.find_max(self.optimality_graph)
        for src, dst_list in self.optimality_graph.items():
            if self.reversed_optimality_graph.get(src) is None:
                self.reversed_optimality_graph.setdefault(src, {})
            for dst, val in dst_list.items():
                if self.max_convolution != max_conv or \
                        self.optimality_graph[src][dst] != self.prev_optimality_graph[src][dst]:
                    self.reversed_optimality_graph[src][dst] = \
                        self.local_reversed_convolution(self.optimality_graph[src][dst], max_conv)
        self.max_convolution = max_conv

        # print(self.prev_reversed_optimality_graph)
        self.prev_reversed_optimality_graph = copy.copy(self.reversed_optimality_graph)

    def graph_to_matrix(self, graph):
        switch_ids = graph.keys()
        switch_ids_sorted = sorted(switch_ids)
        last_switch_id = switch_ids_sorted[-1]
        matrix_graph = [[float("Inf")] * last_switch_id] * last_switch_id
        for src, dst_list in graph.items():
            for dst, val in dst_list.items():
                matrix_graph[src-1][dst-1] = graph.get(src).get(val)
        return matrix_graph

    def find_optimal_path(self, src, dst):
        # graph = self.graph_to_matrix(self.reversed_optimality_graph)
        graph = self.prev_reversed_optimality_graph
        if len(graph.keys()) == 0:
            graph = self.awareness.dict_graph
            
        distance, predecessor = dict(), dict()
        for node in graph:
            distance[node], predecessor[node] = float('inf'), None
        distance[src] = 0

        for _ in range(len(graph) - 1):
            for node in graph:
                for neighbour in graph[node]:
                    if distance[neighbour] > distance[node] + graph[node][neighbour]:
                        distance[neighbour] = distance[node] + graph[node][neighbour]
                        predecessor[neighbour] = node
            # Step 3: Check for negative weight cycles
        for node in graph:
            for neighbour in graph[node]:
                assert distance[neighbour] <= distance[node] + graph[node][neighbour], "Negative weight cycle."
        # print(distance)
        # print(predecessor)

        current = dst
        path = []
        while current is not None:
            path.append(current)
            if predecessor.get(current) is None:
                break
            current = predecessor[current]

        rev_path = list(reversed(path))
        print("path -> %s" % rev_path)
        return rev_path

    def calculate_metrics(self):
        self.all_stats_updated = (self.bandwidth_updated and self.delay_updated and self.speed_updated)
        # print(self.bandwidth_updated , self.delay_updated, self.speed_updated)
        if self.all_stats_updated is True:
            self.calc_optimal_criterias()
            self.calc_convolutions()
            self.calc_reversed_convolutions()
            # self.find_optimal_path(6)
            print('stats calculated')
            self.clear_stats()

    def need_to_update(self, prev_val, val):
        prev_val_more = prev_val + prev_val / 15
        prev_val_less = prev_val - prev_val / 15
        return val <= prev_val_less or val >= prev_val_more

    def clear_stats(self):
        self.all_stats_updated = self.bandwidth_updated = False
        self.delay_updated = self.speed_updated = False
        # print(' we are here ', self.updated_delay_graph)

    def bandwidth_updating(self):
        self.bandwidth_updated = False

    def bandwidth_is_updated(self):
        self.bandwidth_updated = True

    def delay_updating(self):
        self.delay_updated = False

    def delay_is_updated(self):
        self.delay_updated = True

    def speed_updating(self):
        self.speed_updated = False

    def speed_is_updated(self):
        self.speed_updated = True
