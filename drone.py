import graph as gp


class Drone:

    def __init__(self, name):
        self.name = name
        self.locations: [gp.Node] = []
        self.cost = 0
        self.path = []


