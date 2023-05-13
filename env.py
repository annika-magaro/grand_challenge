from obstacles import OBSTACLES
from random import randint
class Env:
    def __init__(self, xI, xG, connected=8, size=50, coverage=0.1, clump_size='small'):
        self.x_range = size  # size of background
        self.y_range = size
        self.xI, self.xG = xI, xG
        if connected == 8:
            self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                            (1, 0), (1, -1), (0, -1), (-1, -1)]
        else:
            self.motions = [(-1, 0), (0, 1), (1, 0), (0, -1)]
        self.obs = self.obs_map(coverage)

    def update_obs(self, obs):
        self.obs = obs

    def obs_map(self, coverage):
        """
        Initialize obstacles' positions
        :return: map of obstacles
        """
        x = self.x_range
        y = self.y_range
        obs = set()
        for i in range(x):
            obs.add((i, 0))
        for i in range(x):
            obs.add((i, y - 1))

        for i in range(y):
            obs.add((0, i))
        for i in range(y):
            obs.add((x - 1, i))

        while len(obs) / (x * y) < coverage:
            segment = OBSTACLES[randint(0, len(OBSTACLES) - 1)]

            dx = randint(0, x)
            dy = randint(0, y)
            for obs_x, obs_y in segment:
                if obs_x + dx < x and obs_y + dy < y:
                    obs.add((obs_x + dx, obs_y + dy))

        if self.xI in obs:
            obs.remove(self.xI)
        if self.xG in obs:
            obs.remove(self.xG)
        # for i in range(10, 21):
        #     obs.add((i, 15))
        # for i in range(15):
        #     obs.add((20, i))

        # for i in range(15, 30):
        #     obs.add((30, i))
        # for i in range(16):
        #     obs.add((40, i))
        return obs