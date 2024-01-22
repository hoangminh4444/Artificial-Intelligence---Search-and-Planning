from search.algorithms import State
from search.map import Map
import getopt
import sys
import heapq


def dijkstra(S_init, S_goal, T):
    CLOSED = {}
    OPEN = []
    heapq.heappush(OPEN, S_init)
    CLOSED[S_init.state_hash()] = S_init
    while len(OPEN) > 0:
        n = heapq.heappop(OPEN)
        if n == S_goal:
            return n.get_g(), len(CLOSED)
        for n1 in T.successors(n):
            if n1.state_hash() not in CLOSED:
                heapq.heappush(OPEN, n1)
                CLOSED[n1.state_hash()] = n1
            if n1.state_hash() in CLOSED and n1.get_g() < CLOSED[n1.state_hash()].get_g():
                CLOSED[n1.state_hash()].set_g(n1.get_g())
                heapq.heapify(OPEN)
    
    return -1, len(CLOSED)

def BiBS (start, goal, T):
    OPEN_F = []
    OPEN_B = []
    CLOSED_F = {}
    CLOSED_B = {}
    heapq.heappush(OPEN_F, start)
    heapq.heappush(OPEN_B, goal)
    CLOSED_F[start.state_hash()] = start
    CLOSED_B[goal.state_hash()] = goal 
    u = float("inf")
    while len(OPEN_F) > 0 and len(OPEN_B) > 0:
        if u < OPEN_F[0].get_g() + OPEN_B[0].get_g():
            x = [x for x in CLOSED_F]
            y = [y for y in CLOSED_B]
            l = set(x + y)
            return u, len(l)
        if OPEN_F[0].get_g() < OPEN_B[0].get_g():
            n = heapq.heappop(OPEN_F)
            for n1 in T.successors(n):
                if n1.state_hash() in CLOSED_B:
                    u = min(u, n1.get_g() + CLOSED_B[n1.state_hash()].get_g())
                if n1.state_hash() not in CLOSED_F:
                    heapq.heappush(OPEN_F, n1)
                    CLOSED_F[n1.state_hash()] = n1
                if n1.state_hash() in CLOSED_F and n1.get_g() < CLOSED_F[n1.state_hash()].get_g():
                    CLOSED_F[n1.state_hash()].set_g(n1.get_g())
                    heapq.heapify(OPEN_F)
        else: 
            n = heapq.heappop(OPEN_B)
            for n1 in T.successors(n):
                if n1.state_hash() in CLOSED_F:
                    u = min(u, n1.get_g() + CLOSED_F[n1.state_hash()].get_g())
                if n1.state_hash() not in CLOSED_B:
                    heapq.heappush(OPEN_B, n1)
                    CLOSED_B[n1.state_hash()] = n1
                if n1.state_hash() in CLOSED_B and n1.get_g() < CLOSED_B[n1.state_hash()].get_g():
                    CLOSED_B[n1.state_hash()].set_g(n1.get_g())
                    heapq.heapify(OPEN_B)
    x = [x for x in CLOSED_F]
    y = [y for y in CLOSED_B]
    l = set(x + y)
    return -1, len(l) 


def main():
    """
    Function for testing your A* and Dijkstra's implementation. There is no need to edit this file.
    Run it with a -help option to see the options available. 
    """
    optlist, _ = getopt.getopt(sys.argv[1:], 'h:m:r:', ['testinstances', 'plots', 'help'])

    plots = False
    for o, a in optlist:
        if o in ("-help"):
            print("Examples of Usage:")
            print("Solve set of test instances: main.py --testinstances")
            print("Solve set of test instances and generate plots: main.py --testinstances --plots")
            exit()
        elif o in ("--plots"):
            plots = True
        elif o in ("--testinstances"):
            test_instances = "test-instances/testinstances.txt"
                              
    gridded_map = Map("dao-map/brc000d.map")
    
    nodes_expanded_dijkstra = []    
    nodes_expanded_bibs = []
    
    start_states = []
    goal_states = []
    solution_costs = []
       
    file = open(test_instances, "r")
    
    for instance_string in file:
        list_instance = instance_string.split(",")
        start_states.append(State(int(list_instance[0]), int(list_instance[1])))
        goal_states.append(State(int(list_instance[2]), int(list_instance[3])))
        
        solution_costs.append(float(list_instance[4]))
    file.close()
        
    for i in range(0, len(start_states)):    
        start = start_states[i]
        goal = goal_states[i]
    
        cost, expanded_diskstra = dijkstra(start, goal, gridded_map) # Implement here the call to your Dijkstra's implementation for start, goal, and gridded_map

        nodes_expanded_dijkstra.append(expanded_diskstra)

        if cost != solution_costs[i]:
            print("There is a mismatch in the solution cost found by Dijkstra and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print()

        cost, expanded_astar = BiBS (start, goal, gridded_map) # Implement here the call to your Bi-BS's implementation for start, goal, and gridded_map

        nodes_expanded_bibs.append(expanded_astar)
        
        if cost != solution_costs[i]:
            print("There is a mismatch in the solution cost found by Bi-HS and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print()
    
    if plots:
        from search.plot_results import PlotResults
        plotter = PlotResults()
        plotter.plot_results(nodes_expanded_bibs, nodes_expanded_dijkstra, "Nodes Expanded (Bi-HS)", "Nodes Expanded (Dijkstra)", "nodes_expanded")
    
    print('Finished running all experiments.')

if __name__ == "__main__":
    main()