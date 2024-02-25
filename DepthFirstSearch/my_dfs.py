"""

Depth-First grid planning

See Wikipedia article (https://en.wikipedia.org/wiki/Depth-first_search)

自己实现一下

"""

import math
import matplotlib.pyplot as plt

show_animation = True


class DepthFirstSearchPlanner:

    def __init__(self, ox, oy, reso, rr):
        """
        Initialize grid map for Depth-First planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.reso = reso
        self.rr = rr
        self.calc_obstacle_map(ox, oy)  # 转化为统一的抽象地图，以适配不同的地图分辨率
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, parent_index, parent):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index  # 作为dict的键，方便查询是否存在
            self.parent = parent

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        Depth First search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """
        snode = self.Node(self.calc_xyindex(sx, self.minx),
                          self.calc_xyindex(sy, self.miny), 0.0, -1, None)
        gnode = self.Node(self.calc_xyindex(gx, self.minx),
                          self.calc_xyindex(gy, self.miny), 0.0, -1, None)
        open_set = dict()
        closed_set = dict()
        open_set[self.calc_grid_index(snode)] = snode
        closed_set[self.calc_grid_index(snode)] = snode

        while True:

            if len(open_set) == 0:
                print("Cannot find path")
                exit(0)
            current = open_set.pop(list(open_set.keys())[-1])  # 将open_set作为栈使用
            c_id = self.calc_grid_index(current)

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.minx),
                         self.calc_grid_position(current.y, self.miny), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event:
                                             [exit(0) if event.key == 'escape'
                                              else None])
                plt.pause(0.01)

            # current是否为到达终点
            if current.x == gnode.x and current.y == gnode.y:
                gnode.parent_index = current.parent_index
                gnode.parent = current.parent
                gnode.cost = current.cost
                break

            # 遍历邻居节点
            for i, _ in enumerate(self.motion):
                neighbor = self.Node(current.x + self.motion[i][0],
                                     current.y + self.motion[i][1],
                                     current.cost + self.motion[i][2],
                                     c_id, None)
                n_id = self.calc_grid_index(neighbor)
                # 判断是否越界和碰撞
                if not self.verify_node(neighbor):
                    continue

                if n_id not in closed_set:
                    open_set[n_id] = neighbor
                    closed_set[n_id] = neighbor
                    neighbor.parent = current

        rx, ry = self.calc_final_path(gnode, closed_set)
        return rx, ry

    def calc_final_path(self, gnode, closedset):
        # 回溯路径
        rx, ry = [self.calc_grid_position(gnode.x, self.minx)], [
            self.calc_grid_position(gnode.y, self.miny)]
        n = gnode.parent
        while n is not None:
            rx.append(self.calc_grid_position(n.x, self.minx))
            ry.append(self.calc_grid_position(n.y, self.miny))
            n = n.parent
        return rx, ry

    def calc_grid_position(self, index, minp):
        """
        calc grid position

        :param index:
        :param minp:
        :return:
        """
        pos = index * self.reso + minp
        return pos

    def calc_xyindex(self, position, min_pos):
        """将实际地图坐标转换到抽象地图的坐标系下
        """
        pos = round((position - min_pos) / self.reso)
        return pos

    def calc_grid_index(self, node):
        """
        这个原点是随机取出来的, 只要保证计算所有node的index是唯一即可
        """
        originx = -1
        originy = -1
        index = (node.y - originy) * self.xwidth + node.x - originx
        return index

    def verify_node(self, node):
        """
            检查是否越界和是否为障碍物
        """
        if node.x < 0 or node.x >= self.xwidth:
            return False
        if node.y < 0 or node.y >= self.ywidth:
            return False
        # collision check
        if self.obmap[node.x][node.y]:
            return False
        return True

    def calc_obstacle_map(self, ox, oy):
        """
            计算抽象的地图,用来规划
        """
        self.minx = round(min(ox))
        self.miny = round(min(oy))
        self.maxx = round(max(ox))
        self.maxy = round(max(oy))
        print("min_x:", self.minx)
        print("min_y:", self.miny)
        print("max_x:", self.maxx)
        print("max_y:", self.maxy)

        self.xwidth = round((self.maxx - self.minx) / self.reso)
        self.ywidth = round((self.maxy - self.miny) / self.reso)
        print("x_width:", self.xwidth)
        print("y_width:", self.ywidth)

        # obstacle map generation
        self.obmap = [[False for _ in range(self.ywidth)]
                      for _ in range(self.xwidth)]
        for ix in range(self.xwidth):
            x = self.calc_grid_position(ix, self.minx)
            for iy in range(self.ywidth):
                y = self.calc_grid_position(iy, self.miny)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obmap[ix][iy] = True
                        break

    def get_motion_model(self):
        # dx, dy, cost,逆时针8个点，作为邻居
        motion = [
            [0, 1, 1],
            [-1, 1, math.sqrt(2)],
            [-1, 0, 1],
            [-1, -1, math.sqrt(2)],
            [0, -1, 1],
            [1, -1, math.sqrt(2)],
            [1, 1, math.sqrt(2)],
            [1, 0, 1]
        ]
        return motion


def generate_map(difficult=True):
    # set obstacle positions
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10.0)
    for i in range(-10, 60):
        ox.append(60.0)
        oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60.0)
    for i in range(-10, 61):
        ox.append(-10.0)
        oy.append(i)
    for i in range(-10, 40):
        ox.append(20.0)
        oy.append(i)
    for i in range(0, 40):
        ox.append(40.0)
        oy.append(60.0 - i)

    return ox, oy


def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    grid_size = 2.0  # [m]
    robot_radius = 1.0  # [m]

    # generate obstacle map
    ox, oy = generate_map()

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    dfs = DepthFirstSearchPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = dfs.planning(sx, sy, gx, gy)

    if show_animation:  # pragma: no cover
        for i in range(len(rx)):
            plt.plot([rx[i]], [ry[i]], "or")
            plt.pause(0.1)
        # plt.plot(rx, ry, "-r")
        plt.pause(0.01)
        plt.show()


if __name__ == '__main__':
    main()
