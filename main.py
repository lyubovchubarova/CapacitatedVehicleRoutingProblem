from typing import List
import math
import pathlib
from collections import defaultdict
import numpy as np
import time
import pandas as pd
from sklearn.metrics import mean_absolute_percentage_error
from tqdm import tqdm
from copy import copy

from solution_methods.utils import read_file
from solution_methods.simulated_annealing import SimulatedAnnealing
from solution_methods.local_optimisation import LocalOptimisation

### basic greedy ###
from solution_methods.greedy_solution import GreedyCVRPSolution

### basic polar angle with greedy ###
from solution_methods.geooptimisation import GeoOptimisation

class GeoOoptimisationCVRP_CONSTRUCT_Solituion(GeoOptimisation):

    def get_local_optimiser(self):
        return LocalOptimisation(self.calculate_route_distance)

    # жадный алгоритм
    def solve_greedy_for_group(self,
                            group: List[int]) -> List[int]:
        depot = self.nodes[self.depot_idx]
        group_nodes = [self.nodes[idx] for idx in group]

        current_node = depot
        visited = {depot.idx}
        route = [depot.idx]

        while len(visited) < len(group_nodes) + 1:
            next_node = min(
                (node for node in group_nodes if node.idx not in visited),
                key=lambda node: math.dist((current_node.x, current_node.y), (node.x, node.y)),
            )
            route.append(next_node.idx)
            visited.add(next_node.idx)
            current_node = next_node

        route.append(depot.idx)
        return route
    
    # отжиг
    def solve_sa_for_group(self, group: List[int]) -> List[int]:
        sa = SimulatedAnnealing(self.nodes, self.depot_idx)
        return sa.solution(group)
    
    def solution(self, 
                 solve_method_for_group: str,
                 optimisation_method: str = None,
                 two_steps: bool = False):
        
        local_optimiser = self.get_local_optimiser()
        groups = self.group_clients_by_vehicle()
        
        all_routes = []
        for group in groups:
            if solve_method_for_group == "greedy":
                route = self.solve_greedy_for_group(group)
            if solve_method_for_group == "simulated_annealing":
                route = self.solve_sa_for_group(group)

            if optimisation_method:
                if optimisation_method == "two_opt":
                    route = local_optimiser.two_opt(route)
                if optimisation_method == "three_opt":
                    route = local_optimiser.three_opt(route)
                if optimisation_method == "or_opt":
                    route = local_optimiser.or_opt(route)
            all_routes.append(route)

        if two_steps == True:
            groups = self.relocate_between_groups(groups)

            for _ in range(10):
                all_routes = []
                for group in groups:
                    if solve_method_for_group == "greedy":
                        route = self.solve_greedy_for_group(group)
                    if solve_method_for_group == "simulated_annealing":
                        route = self.solve_sa_for_group(group)

                    if optimisation_method:
                        if optimisation_method == "two_opt":
                            route = local_optimiser.two_opt(route)
                        if optimisation_method == "three_opt":
                            route = local_optimiser.three_opt(route)
                        if optimisation_method == "or_opt":
                            route = local_optimiser.or_opt(route)
                    all_routes.append(route)

                groups = self.relocate_between_groups(groups)
            
        total_cost = self.calculate_total_cost(all_routes)

        for route in all_routes:
            for node_id in route:
                self.nodes[node_id].visited = True

        valid = self.check_solution_valid()
        if not valid:
            print("НЕВАЛИДНОЕ решение")

        return all_routes, total_cost 
            
    
def optimise_best(all_routes, methods, calculate_cost):

        optimized_routes = []
        for route in all_routes:
            best_cost_route = calculate_cost([route])
            best_route = route

            for method_name, method_func in methods.items():
                if method_name == "swap":
                    continue

                candidate_route = method_func(route)
                candidate_cost = calculate_cost([candidate_route])

                if candidate_cost < best_cost_route:
                    best_cost_route = candidate_cost
                    best_route = candidate_route

            optimized_routes.append(best_route)
        optimized_cost = calculate_cost(optimized_routes)

        swapped_routes = methods["swap"](optimized_routes)
        swapped_cost = calculate_cost(swapped_routes)

        if swapped_cost < optimized_cost:
            final_routes = swapped_routes
            final_cost = swapped_cost
        else:
            final_routes = optimized_routes
            final_cost = optimized_cost

        return final_routes

def construct_name(solve_method_for_group,
                   optimisation_method: str = None,
                   two_steps: bool = False):
    
    method_name = []
    if solve_method_for_group == "greedy":
        method_name.append("Greedy")
    if solve_method_for_group == "simulated_annealing":
        method_name.append("SA_")

    if optimisation_method:
        if optimisation_method == "two_opt":
            method_name.append("2-opt")
        if optimisation_method == "three_opt":
            method_name.append("3-opt")
        if optimisation_method == "or_opt":
            method_name.append("or-opt")
    else:
        method_name.append("NO-opt")

    if two_steps:
        method_name.append("2-steps")
    else:
        method_name.append("1-step")

    return '_'.join(method_name)

if __name__ == "__main__":

    set_a = pathlib.Path("/Users/liubovchubaroba/Downloads/A")
    set_m = pathlib.Path("/Users/liubovchubaroba/Downloads/M")
    set_p = pathlib.Path("/Users/liubovchubaroba/Downloads/P")
    
    # solve_methods_for_group = ["simulated_annealing"]
    # # optimisation_methods_for_local_optimisation = [None, "two_opt", "three_opt", "or_opt"]
    # optimisation_methods_for_local_optimisation = ["or_opt"]
    # two_steps_indicators = [True]

    solve_methods_for_group = ["simulated_annealing"]
    optimisation_methods_for_local_optimisation = ["two_opt", "three_opt", "or_opt"]
    # optimisation_methods_for_local_optimisation = [None]
    two_steps_indicators = [True]

    post_optimization_methods = ["2-opt", 
                            "3-opt", 
                            "Or-opt", 
                            "Swap", 
                            "ALL"]
    
    results = []

    for set_path in [set_a, set_m, set_p]:
        paired_pathes = defaultdict(lambda: {"vrp": False, 
                                             "sol": False})

        for filepath in set_path.iterdir():
            if filepath.suffix == ".vrp":
                paired_pathes[filepath.stem]["vrp"] = True

            if filepath.suffix == ".sol":
                paired_pathes[filepath.stem]["sol"] = True

        for paired_path in tqdm(paired_pathes, desc=f"{set_path.name}"):
            if paired_pathes[paired_path]["vrp"] and paired_pathes[paired_path]["sol"]:

                full_path = set_path / f"{paired_path}.vrp"

                k, capacity, nodes, depot_idx = read_file(full_path)
                
                basic_greedy_solver = GreedyCVRPSolution(k, capacity, nodes, depot_idx)
                basic_optimised_solver = GeoOoptimisationCVRP_CONSTRUCT_Solituion(k, capacity, nodes, depot_idx)


                local_post_optimiser = basic_optimised_solver.get_local_optimiser()

                post_optimiser_methods = {
                    "2-opt": local_post_optimiser.two_opt,
                    "3-opt": local_post_optimiser.three_opt,
                    "Or-opt": local_post_optimiser.or_opt,
                    "swap": local_post_optimiser.swap,
                }

                # оптимальное значение стоимости из .sol
                full_path_sol = set_path / f"{paired_path}.sol"
                with open(full_path_sol) as f:
                    for line in f:
                        line = line.strip()
                        if line.startswith("Cost"):
                            optimal_cost = int(line.split(" ")[-1])
                
                start_time = time.time()
                total_cost = basic_greedy_solver.solution()
                end_time = time.time()
                # if paired_path == "A-n32-k5":
                #     visualize_solution(nodes, basic_greedy_solver.trucks, depot_idx, save_gif=True, 
                #                   gif_name=f"basic_greedy_{paired_path}_solution.gif")

                results.append({
                                "set_name": set_path.name, 
                                "method_name": "Basic Greedy", 
                                "y_true": optimal_cost, 
                                "y_pred": total_cost, 
                                "time": end_time - start_time
                            })
                

                # geo solver without post-optimization
                for solve_method in solve_methods_for_group:
                    for optimisation_method in optimisation_methods_for_local_optimisation:
                        for two_steps in two_steps_indicators:
                            
                            name = construct_name(solve_method, optimisation_method, two_steps)
                            
                            ## current time

                            start_time = time.time()
                            routes, cost = basic_optimised_solver.solution(solve_method, optimisation_method, two_steps)
                            end_time = time.time()

                            # if paired_path == "A-n32-k5":
                            #     basic_optimised_solver.visualize_solution(routes, save_gif=True, gif_name=f"{name}_{paired_path}_solution.gif")
                                           

                            results.append({
                                "set_name": set_path.name, 
                                "method_name": name, 
                                "y_true": optimal_cost, 
                                "y_pred": cost, 
                                "time": end_time - start_time
                            })
                            

                            for method in post_optimization_methods:
                                # print(method)

                                optimized_routes = []
                                if method == "2-opt":
                                    optimized_routes = [local_post_optimiser.two_opt(route) for route in routes]
                                    results.append({
                                        "set_name": set_path.name, 
                                        "method_name": f"{name}_post2opt", 
                                        "y_true": optimal_cost, 
                                        "y_pred": cost, 
                                        "time": end_time - start_time
                                    })
                                elif method == "3-opt":
                                    optimized_routes = [local_post_optimiser.three_opt(route) for route in routes]
                                    results.append({
                                        "set_name": set_path.name, 
                                        "method_name": f"{name}_post3opt", 
                                        "y_true": optimal_cost, 
                                        "y_pred": cost, 
                                        "time": end_time - start_time
                                    })
                                elif method == "Or-opt":
                                    optimized_routes = [local_post_optimiser.or_opt(route) for route in routes]
                                    results.append({
                                        "set_name": set_path.name, 
                                        "method_name": f"{name}_postOropt", 
                                        "y_true": optimal_cost, 
                                        "y_pred": cost, 
                                        "time": end_time - start_time
                                    })
                                elif method == "Swap":
                                    optimized_routes = local_post_optimiser.swap(routes)
                                    results.append({
                                        "set_name": set_path.name, 
                                        "method_name": f"{name}_postSwap", 
                                        "y_true": optimal_cost, 
                                        "y_pred": cost, 
                                        "time": end_time - start_time
                                    })

                                elif method == "ALL":
                                    optimized_routes = copy(routes)
                                    prev_cost = basic_optimised_solver.calculate_total_cost(routes)
                
                                    while True: 
                                        best_routes = optimise_best(
                                            optimized_routes,
                                            post_optimiser_methods,
                                            basic_optimised_solver.calculate_total_cost
                                        )

                                        best_cost = basic_optimised_solver.calculate_total_cost(best_routes)
                        
                                        if best_cost < prev_cost:
                                            optimized_routes = best_routes
                                            prev_cost = best_cost
                                        else:
                                            break

                                    results.append({
                                        "set_name": set_path.name, 
                                        "method_name": f"{name}_postALL", 
                                        "y_true": optimal_cost, 
                                        "y_pred": best_cost, 
                                        "time": end_time - start_time
                                    })

                #     total_cost = geo_greedy_solver.calculate_total_cost(optimized_routes)
                #     total_cost_sa = geo_greedy_solver.calculate_total_cost(optimized_routes_sa)

    df = pd.DataFrame(results)

    df.to_csv("results.csv")
