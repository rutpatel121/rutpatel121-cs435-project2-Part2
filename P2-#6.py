import random


class GridGraph():

    def __init__(self):
        #""" initializes a graph object and create a dictionary"""
        self.graph_dict = {}
        self.maze=[]


        #Adds a new GridNode to the graph with coordinates x and y.
    def addGridNode(self,x,y,node):
        if (x,y,node) not in self.graph_dict:
            self.graph_dict[(x,y,node)] = []

    # """This function adds an undirected edge"""
    def addUndirectedEdge(self,vert1,vert2):
            self.graph_dict[vert1].append(vert2[2])
            self.graph_dict[vert2].append(vert1[2])

        #"""This function removes an undirected edge"""
    def removeUndirectedEdge(self,vert1,vert2):
        if vert2[2] in self.graph_dict[vert1]:
            self.graph_dict[vert1].remove(vert2[2])
        if vert1[2] in self.graph_dict[vert1]:
            self.graph_dict[vert2].remove(vert1[2])


        #"""This returns a set of all Nodes in the graph"""
    def getAllNodes(self):
        return list(self.graph_dict.keys())
maze=None

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


class Main():

    @staticmethod
    def createRandomGridGraph(n):
        graph=GridGraph()
        i=0
        m=0
        while i<=n:
            k=0
            graph.maze.append([])
            while k<=n:
                graph.addGridNode(k,i,m)
                rand = random.randrange(0, 2, 1)
                graph.maze[i].append(0)
                m=m+1
                k=k+1
            i=i+1
        i = 0
        m=0
        while i <= n-1:
            k = 0
            while k <= n-1:
                random_edges=random.randrange(0, 2, 1)
                if random_edges > 0:
                    graph.addUndirectedEdge((k,i,m),(k+1,i,m+1))
                if random_edges > 1:
                    graph.addUndirectedEdge((k,i,m),(k,i+1,m+n))
                m=m+1
                k = k + 1
            m=m+1
            i = i + 1
        return graph

    @staticmethod
    def astar( start, end):

        # Create start and end node
        start_node = Node(None, start)
        start_node.g = start_node.h = start_node.f = 0
        end_node = Node(None, end)
        end_node.g = end_node.h = end_node.f = 0

        # Initialize both open and closed list
        open_list = []
        closed_list = []

        # Add the start node
        open_list.append(start_node)

        # Loop until you find the end
        while len(open_list) > 0:

            # Get the current node
            current_node = open_list[0]
            current_index = 0
            for index, item in enumerate(open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index

            # Pop current off open list, add to closed list
            open_list.pop(current_index)
            closed_list.append(current_node)

            # Found the goal
            if current_node == end_node:
                path = []
                current = current_node
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                return path[::-1]  # Return reversed path

            # Generate children
            children = []
            for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1),
                                 (1, 1)]:  # Adjacent squares

                # Get node position
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

                # Make sure within range
                if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (
                        len(maze[len(maze) - 1]) - 1) or node_position[1] < 0:
                    continue

                # Make sure walkable terrain
                if maze[node_position[0]][node_position[1]] != 0:
                    continue

                # Create new node
                new_node = Node(current_node, node_position)

                # Append
                children.append(new_node)

            # Loop through children
            for child in children:

                # Child is on the closed list
                for closed_child in closed_list:
                    if child == closed_child:
                        continue

                # Create the f, g, and h values
                child.g = current_node.g + 1
                child.h = ((child.position[0] - end_node.position[0]) ** 2) + (
                            (child.position[1] - end_node.position[1]) ** 2)
                child.f = child.g + child.h

                # Child is already in the open list
                for open_node in open_list:
                    if child == open_node and child.g > open_node.g:
                        continue

                # Add the child to the open list
                open_list.append(child)

if __name__ == "__main__":
    main_obj=Main()
    grid_graph=main_obj.createRandomGridGraph(100)
    maze=grid_graph.maze



    print()
    print("****************Grid Graph*****************")
    print("")
    print("A* algorithm------->",    main_obj.astar((0,0),(100,100)))
    print()
