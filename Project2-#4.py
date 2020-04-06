import random
from copy import copy, deepcopy
from collections import deque


class DirectedGraph():


    def __init__(self):
        #""" initializes a graph object and create a dictionary"""
        self.graph_dict = {}


    #This function adds a new node to the graph
    def addNode(self, vertex):
        if vertex not in self.graph_dict:
            self.graph_dict[vertex] = []


    #This adds a directed edge between first and second node
    def addDirectedEdge(self,vert1,vert2):
        test_graph = deepcopy(self.graph_dict)
        test_graph[vert1].append(vert2)
        is_valid, message = self.validate_acyclic(test_graph)
        if is_valid:
            self.graph_dict[vert1].append(vert2)

    def itervalues(self,d, **kwargs): # **kwargs allows you to pass keyworded variable length of arguments to a function.
        return iter(d.values(**kwargs))

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
    def removeDirectedEdge(self,vert1,vert2):
        if vert2 in self.graph_dict[vert1]:
            self.graph_dict[vert1].remove(vert2)


    #This returns a set of all Nodes in the graph
    def getAllNodes(self):
        return list(self.graph_dict.keys())
class Main():


    #Creates n random nodes with randomly assigned unweighted, directed edges.
    @staticmethod
    def createRandomDAGIter(n):
        graph = DirectedGraph()
        i=1
        nodeVal=1
        graph.addNode(1)
        while nodeVal<=n:
            if nodeVal+1<=n:
                graph.addNode(nodeVal+1)
            if nodeVal+2<=n:
                graph.addNode(nodeVal+2)
            if nodeVal+3<=n:
                graph.addNode(nodeVal+3)
            for l in range(3):
                nodeVal = nodeVal + 1
                if nodeVal<=n:
                    graph.addDirectedEdge(i,nodeVal)

            i=i+1
        return graph


class TopSort():

    @staticmethod
    def Kahns(graph):
        Sorted_List = []
        ZeroInDegreeVertex = []
        inDegree = {nodes: 0 for nodes in graph}

        for nodes in graph:
            for edges in graph[nodes]:
                inDegree[edges] += 1

        for nodeVal in inDegree:
            if (inDegree[nodeVal] == 0):
                ZeroInDegreeVertex.append(nodeVal)

        while ZeroInDegreeVertex:
            edges = ZeroInDegreeVertex.pop(0)
            Sorted_List.append(edges)

            for neighbour in graph[edges]:
                inDegree[neighbour] -= 1
                if (inDegree[neighbour] == 0):
                    ZeroInDegreeVertex.append(neighbour)

        return Sorted_List


    @staticmethod
    def mDFS(graph):
        order, enter, state = deque(), set(graph), {}

        def dfs(vertex):
            state[vertex] = 0
            for nodeVal in graph.get(vertex, ()):
                sk = state.get(nodeVal, None)
                if sk == 0: raise ValueError("cycle")
                if sk == 1: continue
                enter.discard(nodeVal)
                dfs(nodeVal)
            order.appendleft(vertex)
            state[vertex] = 1

        while enter: dfs(enter.pop())
        return list(order)

if __name__ == "__main__":
    main_obj=Main()
    DAG=main_obj.createRandomDAGIter(1000)
    sort=TopSort()

    print("****************Directed Acyclic Graph******************")
    print()
    print("Graph Adjacency List------>",DAG.graph_dict)
    print()
    print("Toplogical Sort Kahn’s Algorithm-------->",sort.Kahns(DAG.graph_dict))
    print()
    print("Toplogical Sort mDFS Algorithm------>",sort.mDFS(DAG.graph_dict))
