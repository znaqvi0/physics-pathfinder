class Family:
    def __init__(self, population, sigma, sigma_rate):
        self.family_score = 0
        self.paths = []
        self.generation = 1
        self.population = population
        self.sigma = sigma
        self.best_path = None
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

    def get_average_length(self):
        return sum([path.length() for path in self.paths]) / len(self.paths)

    def calculate_family_score(self):
        return -self.get_average_length()

    def update(self):
        for path in self.paths:
            path.update()

    def next_gen(self):
        self.paths = sorted(self.paths, key=lambda x: x.fitness, reverse=True)
        self.generation += 1

        self.family_score = self.calculate_family_score()

        num_children = self.population // 2

        self.paths = self.paths[0:max(num_children, 1)]
        self.best_path = self.paths[0]
        new_paths = []
        for path in self.paths:
            for j in range(self.population // len(self.paths)):
                new_paths.append(path.varied_copy(self.sigma))

        self.sigma *= self.sigma_rate
        return new_paths
