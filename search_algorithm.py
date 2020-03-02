#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2019, AAIR Lab, ASU"
__authors__ = ["Naman Shah"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Pulkit Verma", "Abhyudaya Srinet"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import heapq
import problem 
import rospy
from std_msgs.msg import String
import argparse
import time
import Queue as queue
import math

rospy.init_node("search_algorithms")
publisher = rospy.Publisher("/actions", String, queue_size=10)
parser = argparse.ArgumentParser()
parser.add_argument('-a', help="Please mention algorithm to use. Possible arguments = {bfs, ucs, gbfs, astar}. Default value is bfs.", metavar='bfs', action='store', dest='algorithm', default="bfs", type=str)
parser.add_argument('-c', help="Use custom heuristic function. No value needed.", action='store_true', dest='custom_heuristic')


def bfs(use_custom_heuristic):
    '''
    Perform BFS to find sequence of actions from initial state to goal state
    '''
    helper = problem.Helper()
    init_state = helper.get_initial_state()
    goal_state = helper.get_goal_state()
    possible_actions = helper.get_actions() 
    action_list = []

    # to get the possible action->(state,cost) mapping from current state
    state_dictionary = helper.get_successor(init_state)
    L=queue.Queue(maxsize=0)
    L.put([["Mv"],init_state])
    visited=[]

    result=False

    while not L.empty():
        current=L.get()
        if current[1].x==goal_state.x and current[1].y==goal_state.y:
            action_list=current[0]
            action_list.pop(0)
            result=True
            break
        elif current[1].x<0 or current[1].y<0:
            #print('state out of bound')
            continue

        elif current[1] in visited:
            continue

        else:
            #print('New State')
            next_states=helper.get_successor(current[1])
            visited.append(current[1])
            for key,value in next_states.items():
                path_list=current[0]
                path_list=path_list+[key]
                L.put([path_list,value[0]])


    if not result:
        action_list=[]



    '''
    YOUR CODE HERE
    '''

    return action_list


def ucs(use_custom_heuristic):
    '''
    Perform UCS to find sequence of actions from initial state to goal state
    '''
    helper = problem.Helper()
    init_state = helper.get_initial_state()
    goal_state = helper.get_goal_state()
    possible_actions = helper.get_actions() 
    action_list = []

    # to get the possible action->(state,cost) mapping from current state
    state_dictionary = helper.get_successor(init_state)

    L_heap = []
    heapq.heappush(L_heap, (0,0, ["mv"],init_state))

    #L.put([["Mv"],init_state])
    visited=[]

    result=False
    counter=0
    while L_heap:
        #current=L.get()
        current=heapq.heappop(L_heap)
        if current[3].x==goal_state.x and current[3].y==goal_state.y:


            action_list=current[2]
            action_list.pop(0)
            result=True
            break
        elif current[3].x<0 or current[3].y<0:
            #print('state out of bound')
            continue
        elif current[3] in visited:
            #print('Already Visited')
            continue
        else:
            #print('New State')
            next_states=helper.get_successor(current[3])
            visited.append(current[3])
            for key,value in next_states.items():
                path_list=current[2]
                path_list=path_list+[key]
                total_cost=current[0]+value[1]
                counter=counter+1

                heapq.heappush(L_heap, (total_cost,counter, path_list, value[0]))

    if not result:

        action_list=[]

    '''
    YOUR CODE HERE
    '''

    return action_list

def gbfs(use_custom_heuristic):
    '''
    Perform GBFS to find sequence of actions from initial state to goal state
    '''
    helper = problem.Helper()
    init_state = helper.get_initial_state()
    goal_state = helper.get_goal_state()
    possible_actions = helper.get_actions() 
    action_list = []


    # to get the possible action->(state,cost) mapping from current state
    state_dictionary = helper.get_successor(init_state)
    if use_custom_heuristic == True:

        def heuristic(state):
            a = goal_state.x - state.x + goal_state.y - state.y
            if state.orientation in ["WEST","SOUTH"]: a = a+1
            else: a=a-1

            return a

        L_heap = []
        heapq.heappush(L_heap, (heuristic(init_state),0, ["mv"], init_state))


        visited = []

        result = False
        counter=0
        while L_heap:
            current = heapq.heappop(L_heap)
            if current[3].x==goal_state.x and current[3].y==goal_state.y:

                action_list = current[2]
                action_list.pop(0)
                result = True
                break
            elif current[3].x < 0 or current[3].y < 0:

                continue
            elif current[3] in visited:

                continue

            else:

                next_states = helper.get_successor(current[3])
                visited.append(current[3])
                for key, value in next_states.items():
                    path_list = current[2]
                    path_list = path_list + [key]
                    hrc_val=heuristic(current[3])
                    counter=counter+1
                    heapq.heappush(L_heap, (hrc_val,counter, path_list, value[0]))

        if not result:

            action_list = []

    else:

        def heuristic(state):
            a=goal_state.x-state.x+goal_state.y-state.y
            return a

        L_heap = []
        heapq.heappush(L_heap, (heuristic(init_state),0, ["mv"], init_state))

        visited = []

        result = False
        counter = 0
        while L_heap:
            current = heapq.heappop(L_heap)
            if current[3].x==goal_state.x and current[3].y==goal_state.y:

                action_list = current[2]
                action_list.pop(0)
                result = True
                break
            elif current[3].x < 0 or current[3].y < 0:
                #print('state out of bound')
                continue
            elif current[3] in visited:
                #print('Already Visited')
                continue

            else:
                #print('New State')
                next_states = helper.get_successor(current[3])
                visited.append(current[3])
                for key, value in next_states.items():
                    counter=counter+1
                    path_list = current[2]
                    path_list = path_list + [key]
                    hrc_val = heuristic(current[3])

                    heapq.heappush(L_heap, (hrc_val,counter, path_list, value[0]))

        if not result:
            action_list = []
    '''
    YOUR CODE HERE
    '''

    return action_list

def astar(use_custom_heuristic):
    '''
    Perform A* search to find sequence of actions from initial state to goal state
    '''
    helper = problem.Helper()
    init_state = helper.get_initial_state()
    goal_state = helper.get_goal_state()
    possible_actions = helper.get_actions() 
    action_list = []


    # to get the possible action->(state,cost) mapping from current state
    state_dictionary = helper.get_successor(init_state)
    if use_custom_heuristic == True:

        def heuristic(state):
            a=goal_state.x - state.x + goal_state.y - state.y
            if state.orientation in ['WEST','SOUTH']:a=a+1
            else: a=a-1
            return a

        L_heap = []
        heapq.heappush(L_heap, (heuristic(init_state),0, ["mv"], init_state,0))

        visited = []

        result = False
        counter=0
        while L_heap:
            current = heapq.heappop(L_heap)
            if current[3].x==goal_state.x and current[3].y==goal_state.y:


                action_list = current[2]
                action_list.pop(0)
                result = True
                break
            elif current[3].x < 0 or current[3].y < 0:
                #print('state out of bound')
                continue
            elif current[3] in visited:
                #print('Already Visited')
                continue

            else:
                #print('New State')
                next_states = helper.get_successor(current[3])
                visited.append(current[3])
                for key, value in next_states.items():
                    
                    path_list = current[2]
                    path_list = path_list + [key]
                    counter=counter+1
                    total_cost=current[4]+value[1]
                    astar_val=heuristic(current[3])+total_cost

                    heapq.heappush(L_heap, (astar_val,counter, path_list, value[0],total_cost))

        if not result:

            action_list = []

    else:

        def heuristic(state):
            a=goal_state.x-state.x+goal_state.y-state.y
            return a

        L_heap = []
        heapq.heappush(L_heap, (heuristic(init_state),0, ["mv"], init_state,0))

        visited = []

        result = False
        counter = 0
        while L_heap:
            current = heapq.heappop(L_heap)
            if current[3].x==goal_state.x and current[3].y==goal_state.y:


                action_list = current[2]
                action_list.pop(0)
                result = True
                break
            elif current[3].x < 0 or current[3].y < 0:
                #print('state out of bound')
                continue
            elif current[3] in visited:
                # print('Already Visited')
                continue

            else:
                #print('New State')
                next_states = helper.get_successor(current[3])
                visited.append(current[3])
                for key, value in next_states.items():
                    counter=counter+1
                    path_list = current[2]
                    path_list = path_list + [key]
                    total_cost=current[4]+value[1]
                    astar_val=heuristic(current[3])+total_cost

                    heapq.heappush(L_heap, (astar_val,counter, path_list, value[0],total_cost))

        if not result:

            action_list = []
    '''
    YOUR CODE HERE
    '''

    return action_list


def exec_action_list(action_list):
    '''
    publishes the list of actions to the publisher topic
    action_list: list of actions to execute
    '''
    plan_str = '_'.join(action for action in action_list)
    publisher.publish(String(data = plan_str))


if __name__ == "__main__":
    # DO NOT MODIFY BELOW CODE
    args = parser.parse_args()
    algorithm = globals().get(args.algorithm)
    if algorithm is None:
        print "Incorrect Algorithm name."
        exit(1)
    if args.algorithm in ["bfs", "ucs"] and args.custom_heuristic == True:
        print ("Error: "+args.algorithm+" called with heuristic")
        exit(1)

    start_time = time.time()
    actions = algorithm(args.custom_heuristic)
    time_taken = time.time() - start_time
    print("Time Taken = " + str(time_taken))
    print("Plan = " + str(actions))
    exec_action_list(actions)
