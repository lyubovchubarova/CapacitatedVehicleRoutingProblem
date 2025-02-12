import math
from typing import List
from solution_methods.utils import read_file
import pathlib
from collections import defaultdict
import numpy as np
from copy import copy
from .utils import Solution

class GeoOptimisation(Solution):

    def __init__(self,
                 k,
                 capacity,
                 nodes,
                 depot_idx):

        self.nodes = nodes
        for node in self.nodes:
            self.nodes[node].visited = False

        self.depot_idx = depot_idx

        self.k = k
        self.capacity = capacity

    def calculate_polar_angle(self,
                              node,
                              depot) -> float:
        dx = node.x - depot.x
        dy = node.y - depot.y
        return math.atan2(dy, dx)
    
    def calculate_route_distance(self, route: List[int]) -> float:
        distance = 0
        for i in range(len(route) - 1):
            start_node = self.nodes[route[i]]
            end_node = self.nodes[route[i + 1]]
            distance += math.dist((start_node.x, start_node.y), (end_node.x, end_node.y))
        return distance
    
    def calculate_total_cost(self,
                             routes: List[List[int]]) -> float:
        total_cost = 0

        for route in routes:
            # for i in range(len(route) - 1):
            #     start_node = self.nodes[route[i]]
            #     end_node = self.nodes[route[i + 1]]
                total_cost += self.calculate_route_distance(route)

        return total_cost
    
    # итеративно перераспределяем клиентов так, чтобы общая стоимость уменьшилась
    def relocate_between_groups(self, groups):
        for i in range(len(groups)):
            for j in range(len(groups)):
                if i == j:
                    continue
                for client in groups[i]:
                    if sum(self.nodes[c].demand for c in groups[j]) + self.nodes[client].demand <= self.capacity:
                        new_group_i = groups[i].copy()
                        new_group_j = groups[j].copy()
                        new_group_i.remove(client)
                        new_group_j.append(client)
                        if self.calculate_total_cost([new_group_i, new_group_j]) < self.calculate_total_cost([groups[i], groups[j]]):
                                groups[i], groups[j] = new_group_i, new_group_j
        return groups
    
    def group_clients_by_vehicle(self) -> List[List[int]]:
        depot = self.nodes[self.depot_idx]
        clients = [node for idx, node in self.nodes.items() if idx != self.depot_idx]

        # рассчет полярного угла и сортировка
        for client in clients:
            client.angle = self.calculate_polar_angle(client, depot)

        clients_sorted = sorted(clients, key=lambda node: node.angle)

        groups = []
        used_clients = set()

        # распределяем клиентов в K групп
        for _ in range(self.k):
            current_group = []
            current_capacity = 0
            for client in clients_sorted:
                if client.idx in used_clients:
                    continue
                if current_capacity + client.demand <= self.capacity:
                    current_group.append(client.idx)
                    current_capacity += client.demand
                    used_clients.add(client.idx)
            if current_group:
                groups.append(current_group)

        # нераспределенные
        remaining_clients = [client for client in clients_sorted if client.idx not in used_clients]

        # пытаемся добавить нераспределенных
        while remaining_clients:
            added_any = False
            # пытаемся добавить каждого оставшегося
            for client in remaining_clients.copy():
                for group in groups:
                    group_capacity = sum(self.nodes[idx].demand for idx in group)
                    if group_capacity + client.demand <= self.capacity:
                        group.append(client.idx)
                        used_clients.add(client.idx)
                        remaining_clients.remove(client)
                        added_any = True
                        break

            # если остались клиенты
            if remaining_clients:
                prev_groups = [g.copy() for g in groups]
                groups = self.relocate_between_groups(groups) # пытаемся перераспределить имеющихся для снижения общей стоимости
                # если изменений нет
                if groups == prev_groups:
                    for client in remaining_clients.copy():
                        # выбираем группу с минимальной загрузкой
                        min_group = min(groups, key=lambda g: sum(self.nodes[idx].demand for idx in g))
                        min_group.append(client.idx)
                        used_clients.add(client.idx)
                        remaining_clients.remove(client)
                    # пытаемся оптимизировать
                    groups = self.relocate_between_groups(groups)
                    break
            else:
                break

        # Проверка, что все клиенты распределены
        if len(used_clients) != len(clients):
            raise ValueError("Не удалось распределить всех клиентов.")
        
        return groups
    
    def check_solution_valid(self) -> bool:
        all_routed = all(self.nodes[node].visited or self.nodes[node].demand == 0 
                         for node in self.nodes)
        return all_routed
