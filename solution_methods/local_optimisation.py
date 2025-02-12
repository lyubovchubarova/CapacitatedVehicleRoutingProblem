from typing import List


class LocalOptimisation:

    def __init__(self, 
                 calculate_route_distance):
        self.calculate_route_distance = calculate_route_distance

    def two_opt(self, route: List[int]) -> List[int]:
        best_route = route[:]
        best_distance = self.calculate_route_distance(route)
        improved = True

        while improved:
            improved = False
            for i in range(1, len(route) - 2):
                for j in range(i + 1, len(route)):
                    if j - i == 1:  # соседние узлы не изменяют маршрут
                        continue
                    new_route = route[:i] + route[i:j][::-1] + route[j:]
                    new_distance = self.calculate_route_distance(new_route)
                    if new_distance < best_distance:
                        best_route = new_route
                        best_distance = new_distance
                        improved = True

            route = best_route

        return best_route

    def three_opt(self, route: List[int]) -> List[int]:
        best_route = route[:]
        best_distance = self.calculate_route_distance(route)
        improved = True

        while improved:
            improved = False
            for i in range(len(route) - 2):
                for j in range(i + 1, len(route) - 1):
                    for k in range(j + 1, len(route)):
                        new_routes = [
                            route[:i] + route[i:j][::-1] + route[j:k][::-1] + route[k:], 
                            route[:i] + route[j:k] + route[i:j] + route[k:], 
                            route[:i] + route[j:k][::-1] + route[i:j][::-1] + route[k:]   
                        ]
                        for new_route in new_routes:
                            new_distance = self.calculate_route_distance(new_route)
                            if new_distance < best_distance:
                                best_route = new_route
                                best_distance = new_distance
                                improved = True

            route = best_route

        return best_route

    def or_opt(self, route: List[int], segment_length=3) -> List[int]:
        best_route = route[:]
        best_distance = self.calculate_route_distance(route)

        for i in range(len(route) - segment_length):
            segment = route[i:i + segment_length]
            remaining_route = route[:i] + route[i + segment_length:]

            for j in range(len(remaining_route) + 1):
                new_route = remaining_route[:j] + segment + remaining_route[j:]
                new_distance = self.calculate_route_distance(new_route)
                if new_distance < best_distance:
                    best_route = new_route
                    best_distance = new_distance

        return best_route

    def swap(self, routes: List[List[int]]) -> List[List[int]]:

        for i, route_a in enumerate(routes):
            for j, client_a in enumerate(route_a):
                for k, route_b in enumerate(routes):
                    if i == k:  # Не обмениваем клиентов в одном маршруте
                        continue
                    for l, client_b in enumerate(route_b):
                        new_route_a = route_a[:j] + [client_b] + route_a[j + 1:]
                        new_route_b = route_b[:l] + [client_a] + route_b[l + 1:]
                        current_distance = sum(self.calculate_route_distance(r) for r in routes)
                        new_distance = (
                            self.calculate_route_distance(new_route_a) +
                            self.calculate_route_distance(new_route_b)
                        )
                        if new_distance < current_distance:
                            routes[i] = new_route_a
                            routes[k] = new_route_b

        return routes