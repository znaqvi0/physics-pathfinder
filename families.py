from engine import Ball
from vectors import Vec
import numpy as np


class Family:
    def __init__(self, population, sigma, sigma_rate):
        self.family_score = 0
        self.paths = []
        self.generation = 1
        self.population = population
        self.sigma = sigma
        self.best_path = Ball(Vec(), Vec(), 1, 1)
        self.last_family = False
        self.sigma_rate = sigma_rate

    def populate(self, seed):
        for i in range(self.population):
            self.add(seed.varied_copy(self.sigma, dropout=False))
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

    def next_gen(self):
        self.paths = sorted(self.paths, key=lambda x: x.fitness, reverse=True)
        self.generation += 1

        avg_score = sum(path.fitness for path in self.paths) / len(self.paths)
        self.family_score = avg_score  # careful with sign

        num_children = self.population // 2

        self.paths = self.paths[0:max(num_children, 1)]
        self.best_path = self.paths[0]
        new_paths = []
        for path in self.paths:
            for j in range(self.population // len(self.paths)):
                new_paths.append(path.varied_copy(self.sigma))

        self.sigma *= self.sigma_rate
        return new_paths
