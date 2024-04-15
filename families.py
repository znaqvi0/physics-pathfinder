from engine import Ball
from vectors import Vec
import numpy as np


class Family:
    def __init__(self, population, sigma, sigma_rate):
        self.family_score = 0
        self.paths = []
        self.generations_passed = 0
        self.population = population
        self.sigma = sigma
        self.best_path = Ball(Vec(), Vec(), 1, 1)
        self.last_family = False
        self.sigma_rate = sigma_rate

    def populate(self, seed):
        for i in range(self.population):
            self.add(seed.varied_copy(self.sigma))
        return self

    def add(self, path):
        self.paths.append(path)

    def sort(self):
        self.paths = sorted(self.paths, key=lambda x: x.fitness)

    def calculate_family_score(self):
        score = sum([path.fitness for path in self.paths]) / len(self.paths)
        return score

    def update(self):
        for path in self.paths:
            path.update()

    def all_done(self):
        for path in self.paths:
            if not path.done:
                return False
        return True

    def tribalism(self, num_families, new_sigma):
        new_families = []
        lst = sorted(self.paths, key=lambda x: x.personality())
        sections = np.array_split(lst, num_families)
        for i in range(len(sections)):  # for every section
            new_families.append(Family(self.population // num_families, new_sigma, self.sigma_rate))
            for ball in sections[i]:  # add every ball in the section to the corresponding family
                new_families[i].add(ball)
        return new_families

    def next_gen(self):
        self.paths = sorted(self.paths, key=lambda x: x.fitness, reverse=True)
        self.generations_passed += 1

        avg_score = sum(path.fitness for path in self.paths) / len(self.paths)
        self.family_score = avg_score  # careful with sign

        num_children = self.population // 5

        self.paths = self.paths[0:max(num_children, 1)]
        self.best_path = self.paths[0]
        new_paths = []
        for path in self.paths:
            for j in range(self.population // len(self.paths)):
                new_paths.append(path.varied_copy(self.sigma))

        self.sigma *= self.sigma_rate
        return new_paths
