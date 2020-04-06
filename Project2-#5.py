import random
from copy import copy, deepcopy
import collections
import heapq
from collections import deque
class WeightedGraph():


    def __init__(self):
        #""" initializes a graph object and create a dictionary"""
        self.graph_dict = {}
        self.edges=[]
        self.n=0 #stores no. of nodes

    #This function adds a new node to the graph
    def addNode(self, vertex):
        if vertex not in self.graph_dict:
            self.graph_dict[vertex] = []
            self.n=self.n+1


    #This adds a directed edge between first and second node
    def addWeightedEdge(self,vert1,vert2,weight):
        test_graph = deepcopy(self.graph_dict)
        test_graph[vert1].append(vert2)
        is_valid, message = self.validate_acyclic(test_graph)
        if is_valid:
            self.graph_dict[vert1].append(vert2)
            self.edges.append((vert1,vert2,weight))

    def itervalues(self,d, **kw):
        return iter(d.values(**kw))

    #Returns a list of all nodes in the graph with no dependencies. Used to validate DAG
    def independent_nodes(self, graph):

        dependent_nodes = set(
            node for dependents in self.itervalues(graph) for node in dependents
        )
        return [node for node in graph.keys() if node not in dependent_nodes]


    #Returns a topological ordering of the DAG. Raises an error if this is not possible (graph is not valid)
    #Used to check either graph is acyclic or not after adding a new edge
    def topological_sort(self, graph):

        in_degree = {}
        for nodes in graph:
            in_degree[nodes] = 0

        for nodes in graph:
            for edges in graph[nodes]:
                in_degree[edges] += 1

        queue = deque()
        for nodes in in_degree:
            if in_degree[nodes] == 0:
                queue.appendleft(nodes)

        l = []
        while queue:
            nodes = queue.pop()
            l.append(nodes)
            for edges in graph[nodes]:
                in_degree[edges] -= 1
                if in_degree[edges] == 0:
                    queue.appendleft(edges)

        if len(l) == len(graph):
            return l
        else:
            raise ValueError('graph is not acyclic')


    # this function checks either DAG is valid or not after adding adge
    def validate_acyclic(self, graph):
        graph = graph if graph is not None else self.graph
        if len(self.independent_nodes(graph)) == 0:
            return (False, 'no independent nodes detected')
        try:
            self.topological_sort(graph)
        except ValueError:
            return (False, 'failed topological sort')
        return (True, 'valid')


    #This removes an directed edge between first and second
    def removeDirectedEdge(self,vert1,vert2,weight):
        if vert2 in self.graph_dict[vert1]:
            self.graph_dict[vert1].remove(vert2)
            self.edges.remove((vert1,vert2,weight))


    #This returns a set of all Nodes in the graph
    def getAllNodes(self):
        return list(self.graph_dict.keys())
weighted_graph=None
class Main():

    @staticmethod
    def createRandomCompleteWeightedGraph(n):
        graph=WeightedGraph()
        i=1
        while i<=n:
            graph.addNode(i)
            i=i+1
        l=1
        while l<=n:
            nodeVal=1+l
            while nodeVal<=n:
                w=random.randrange(1, 10, 1)
                graph.addWeightedEdge(l,nodeVal,w)
                nodeVal=nodeVal+1

            l=l+1
        return graph

    @staticmethod
    def createLinkedList(n):
        graph = WeightedGraph()
        if n>0:
            graph.addNode(1)
            i=2
            while i<=n:
                graph.addNode(i)
                graph.addWeightedEdge(i-1,i,1)
                i=i+1
        return graph

    @staticmethod
    def shortestPath(edges, source, sink):
        # create a weighted DAG - {node:[(cost,neighbour), ...]}
        graph = collections.defaultdict(list)
        for l, secondNodeEdge, weightEdge in edges:
            graph[l].append((weightEdge, secondNodeEdge))
        # create a priority queue and hash set to store visited nodes
        queue, visited = [(0, source, [])], set()
        heapq.heapify(queue)
        # traverse graph with BFS
        while queue:
            (cost, node, path) = heapq.heappop(queue)
            # visit the node if it was not visited before
            if node not in visited:
                visited.add(node)
                path = path + [node]
                # hit the sink
                if node == sink:
                    return cost
                # visit neighbours
                for weightEdge, neighbour in graph[node]:
                    if neighbour not in visited:
                        heapq.heappush(queue, (cost + weightEdge, neighbour, path))
        return float("inf")

    @staticmethod
    def dijkstras(start):
        map_dic={}
        i=1
        while i<=weighted_graph.n:
            if i!=start:
                distance=Main.shortestPath(weighted_graph.edges, start, i)
                map_dic[(start,i)]=distance
            i=i+1
        return map_dic

if __name__ == "__main__":
    main_obj=Main()
    weighted_graph=main_obj.createRandomCompleteWeightedGraph(10)
    linked_list=main_obj.createLinkedList(10)



    print("****************Weighted Graph******************")
    print()
    print("Adjacency list 10 nodes------>",weighted_graph.graph_dict)# Print adjacency list of graph created by createRandomCompleteWeightedGraph() function
    print()
    print("Weighted Edge list------->",weighted_graph.edges) #Print list of edges and weights of graph created by createRandomCompleteWeightedGraph() function
    print()
    print("Dictionary Mapping------>",main_obj.dijkstras(3)) #Print dictionary mapping each Node node in the graph to the minimum value from start to get to node. Last part of 5
    print()
    print("*****************Linked List******************")
    print()
    print("Linked List Adjacency List------>",linked_list.graph_dict)
    print()
    print("Linked List Uniform Edges",linked_list.edges)