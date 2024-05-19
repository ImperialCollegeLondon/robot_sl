import numpy as np
import scipy.spatial.distance as distance
from time import time

class GeneticAlgorithm:
    '''
    Implementation of an integer genetic algorithm.
    Inspired by pymoo: https://pymoo.org/algorithms/soo/ga.html
    
    Member/individuals: (numpy array) [x, fitness, violation]
    '''

    def __init__(self, objective, len_x, 
                constraints=[], pop_size=20, x0=None,
                max_gen=1000, max_eval=np.inf,
                seed=None, mutation_rate=1.0,
                max_no_improvement=150):
        self.objective = objective
        self.constraints = constraints
        self.pop_size = pop_size
        self.len_x = len_x
        self.genealogy = []
        self.rng = np.random.default_rng(seed)
        self.mutation_rate = mutation_rate
        self.max_gen = max_gen
        self.max_eval = max_eval
        self.max_no_improvement = max_no_improvement
        self.best_fitness = np.inf
        self.no_improvement_counter = 0
        self.eval_counter = 0
        self.gen_counter = 0
        if x0 is None:
            x0 = np.arange(len_x)
        assert len(x0) == len_x, 'Invalid initial state size! Got length {}'.format(len(x0))
        self.reset_population(x0)
        # id_order = self.rank_pop()
        # print("Initial fitness: " + str(self.pop_fitness[id_order[0]]))

    def solve(self):
        """
        status: 0 - did not converge, 1 - converged
        return: (x, fitness, status, n_gen, n_eval, time, genealogy)
        """
        status = 0
        start_time = time()

        while status == 0:
            status = self.next_generation()
        time_cost = time() - start_time
        
        id_order = self.rank_pop()
        results = (self.population[id_order[0]][0].tolist(), 
            self.population[id_order[0]][1], 
            status, 
            self.gen_counter, 
            self.eval_counter,
            time_cost,
            self.genealogy)
        return results

    def next_generation(self):
        # mating
        children = []
        while len(children)<self.pop_size:
            parents_ids = self.selection(self.population)
            children_temp = self.crossover_pop(self.population, parents_ids)
            children_temp = self.mutate_pop(children_temp)
            children_temp = self.remove_duplicate(children_temp)
            children_temp = self.remove_duplicate(children_temp, self.population)
            children_temp = self.remove_duplicate(children_temp, children)
            for c in children_temp:
                children.append(c)
        # check termination
        status = self.check_ternimation()
        if status:
            return status
        # evaluate
        children = self.eval_pop(children)
        # merge
        self.population = np.concatenate([self.population, children])
        # remove redundent population
        self.population = self.fitness_survival()
        # increment the generation counter by 1
        self.gen_counter += 1
        return 0

    def mutate_pop(self, pop):
        children = []
        
        for ind in range(len(pop)):
            if(self.rng.random() < self.mutation_rate):
                x = self.mutate(pop[ind][0])
                children.append(np.array([x, np.inf, np.inf], dtype=object))
            else:
                children.append(pop[ind])
        return np.array(children)

    def mutate(self, individual):
        '''
        Insertion mutation
        [1] M. R. Kianifar, F. Campean, and A. Wood, ‘Application of permutation genetic algorithm for sequential 
                model building–model validation design of experiments’, Soft Comput, vol. 20, no. 8, pp. 3023–3044, 
                Aug. 2016, doi: 10.1007/s00500-015-1929-5.
        '''
        section_ends = np.sort(self.rng.choice(len(individual), size=2, replace=False))
        new_section = np.flip(individual[section_ends[0]:section_ends[1]])
        return np.concatenate([individual[:section_ends[0]], new_section, individual[section_ends[1]:]])
        # '''
        # Swap mutation
        # '''
        # section_ends = np.sort(self.rng.choice(len(individual), size=2, replace=False))
        # individual[section_ends] = np.flip(individual[section_ends])
        # return individual

    def crossover_pop(self, pop, parents_ids):
        children = []
        for p in parents_ids:
            x = self.crossover(pop[p[0]][0], pop[p[1]][0])
            children.append(np.array([x[0], np.inf, np.inf], dtype=object))
            children.append(np.array([x[1], np.inf, np.inf], dtype=object))
        return np.array(children)

    def crossover(self, parent1, parent2):
        ''' 
        Ordered crossover for TSP problems: swap part of the genes from parent1 with their order in parent2
        [1] M. R. Kianifar, F. Campean, and A. Wood, ‘Application of permutation genetic algorithm for sequential 
                model building–model validation design of experiments’, Soft Comput, vol. 20, no. 8, pp. 3023–3044, 
                Aug. 2016, doi: 10.1007/s00500-015-1929-5.
        '''        
        section_ends = np.sort(self.rng.choice(len(parent1), size=2, replace=False))
        section_p1 = parent1[section_ends[0]:section_ends[1]]
        offstring_1 = [item for item in parent2 if item not in section_p1]
        section_p2 = parent2[section_ends[0]:section_ends[1]]
        offstring_2 = [item for item in parent1 if item not in section_p2]
        return [np.concatenate([offstring_1[:section_ends[0]], section_p1, offstring_1[section_ends[0]:]]).astype(int), 
                np.concatenate([offstring_2[:section_ends[0]], section_p2, offstring_2[section_ends[0]:]]).astype(int)]

    def compare_x(self, pop, a, b):
        '''
        [1] K. Deb, ‘An efficient constraint handling method for genetic algorithms’, Comput. Methods Appl. Mech. Engrg., p. 28, 2000.
        '''
        if pop[a][2]>pop[b][2]:
            return b
        elif pop[a][2]<pop[b][2]:
            return a
        else:
            return a if pop[a][1]<pop[b][1] else b

    def selection(self, pop):
        n_random = np.ceil(self.pop_size/2).astype(int)*2*2 # garantees to be multiple of 2; binary comparison

        perms = []
        for i in range(np.ceil(n_random / len(pop)).astype(int)):
            perms.append(np.random.permutation(len(pop)))
        P = np.concatenate(perms)[:n_random]

        # get random permutations and reshape them
        P = np.reshape(P, (n_random//2, 2))

        # compare using tournament function
        S = []
        for i in range(n_random//2):
            S.append(self.compare_x(pop, P[i,0], P[i,1]))

        return np.reshape(S, (n_random//4, 2))

    def check_ternimation(self):
        if self.no_improvement_counter>=self.max_no_improvement:
            return 1
        elif self.gen_counter >= self.max_gen:
            return 2
        elif self.eval_counter >= self.max_eval:
            return 3
        else:
            return 0

    def eval_pop(self, population):
        for m in population:
            if m[1] is np.inf:
                m[1] = self.get_fitness(m[0])
                m[2] = self.get_violation(m[0])
        return population

    def get_pop_fitness(self, population):
        return np.array([self.get_fitness(m[0]) for m in population])

    def get_fitness(self, x):
        self.eval_counter += 1
        return self.objective(x)

    def get_violation(self, x):
        # the sum of the contraint violations
        return sum([c(x) for c in self.constraints])

    def rank_pop(self):
        ''' 
        Rank the population accourding to the fitness then the violation.
        return an ordered list of sets with id and fitness
        '''
        return np.lexsort((self.population[:,1],self.population[:,2]))

    def reset_population(self, x0):
        self.population = []
        for _ in range(self.pop_size):
            x = self.gen_random_member(x0)
            self.population.append(np.array([x, self.get_fitness(x), self.get_violation(x)], dtype=object))
        self.population = np.array(self.population)
        self.population = self.remove_duplicate(self.population)
        self.genealogy.append(self.population[np.argmax(self.population[:,1])][0])

    def gen_random_member(self, x):
        x = self.rng.permutation(x)
        return x

    def remove_duplicate(self, pop, pop2=None):
        # ''' 
        # Two individuals are considered the same if their fitness are the same
        # '''
        # pop_fitness = self.get_pop_fitness(pop)
        # _, indices = np.unique(pop_fitness, return_index=True)
        # return pop[indices]    
        pop_x = [i[0].astype(float).tolist() for i in pop]
        if pop2 is None or len(pop2)==0:
            fitness_distance_mat = distance.cdist(pop_x, pop_x)
            fitness_distance_mat[np.triu_indices(len(pop))] = np.inf
        else:
            pop2_x = [i[0].astype(float).tolist() for i in pop2]
            fitness_distance_mat = distance.cdist(pop_x, pop2_x)
        is_duplicate = np.full(len(pop), False)
        is_duplicate[np.any(fitness_distance_mat <= 1e-16, axis=1)] = True
        return pop[~is_duplicate]

    def update_best_fitness(self, best_fitness):
        if best_fitness is not np.inf:
            if np.isclose(best_fitness, self.best_fitness, rtol=0, atol=1e-12):
                self.no_improvement_counter += 1
            else:
                self.no_improvement_counter = 0
        else:
            self.no_improvement_counter = 0
        self.best_fitness = best_fitness

    def fitness_survival(self):
        '''
        reduce population to nominal size by those killing worst fitness
        '''
        rank = self.rank_pop()
        self.update_best_fitness(self.population[rank[0]][1] if self.population[rank[0]][2]==0 else np.inf)
        self.genealogy.append(self.population[rank[0]][0])
        return self.population[rank[:self.pop_size-1]]