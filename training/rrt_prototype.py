import numpy as np
import matplotlib.pyplot as plt
import random

class RRT:
    def __init__(self, start, goal, obstacle_list, rand_area, expand_dis=0.1, path_resolution=0.01, goal_sample_rate=5, max_iter=500):
        self.start = Node(start)
        self.goal = Node(goal)
        self.rand_area = rand_area
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []

    def plan(self, animation=True):
        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_collision(new_node, self.obstacle_list):
                self.node_list.append(new_node)
                if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= self.expand_dis:
                    final_node = self.steer(self.node_list[-1], self.goal, self.expand_dis)
                    if self.check_collision(final_node, self.obstacle_list):
                        return self.generate_final_course(len(self.node_list) - 1)

            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)

        return None  # cannot find path

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = int(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * np.cos(theta)
            new_node.y += self.path_resolution * np.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.x = to_node.x
            new_node.y = to_node.y
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.goal.x, self.goal.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.goal.x
        dy = y - self.goal.y
        return np.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = Node(
                random.uniform(self.rand_area[0], self.rand_area[1]),
                random.uniform(self.rand_area[0], self.rand_area[1]))
        else:  # goal point sampling
            rnd = Node(self.goal.x, self.goal.y)
        return rnd

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, size) in self.obstacle_list:
            self.draw_circle(ox, oy, size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.goal.x, self.goal.y, "xr")
        plt.axis("equal")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def draw_circle(x, y, size):
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * np.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * np.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, "-k")

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        min_ind = dlist.index(min(dlist))

        return min_ind

    @staticmethod
    def check_collision(node, obstacleList):

        if node is None:
            return False

        for (ox, oy, size) in obstacleList:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= size**2:
                return False  # collision

        return True  # safe

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = np.hypot(dx, dy)
        theta = np.arctan2(dy, dx)
        return d, theta


class Node:
    def __init__(self, x, y=None):
        if y is None: # Treat x as a vector
            self.x = x[0]
            self.y = x[1]
        else:
            self.x = x
            self.y = y
        self.path_x = []
        self.path_y = []
        self.parent = None


def main():
    print("start " + __file__)

    # ====Search Path with RRT====
    obstacle_list = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2),
        (8, 10, 1)
    ]  # [x, y, radius]
    # Set Initial parameters
    rrt = RRT(
        start=[0, 0],
        goal=[6, 10],
        rand_area=[-2, 15],
        obstacle_list=obstacle_list,
        expand_dis=0.5,
        max_iter=2000)
    path = rrt.plan(animation=False)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        # Draw final path
        rrt.draw_graph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.01)  # Need for Mac
        plt.savefig('rrt_path.png')
        print("Path saved to rrt_path.png")


if __name__ == '__main__':
    main()
