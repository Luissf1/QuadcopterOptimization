% Clear workspace
clear all
close all

% Problem definition
nVar = 10; % number of variables
VarMin = -3; % lower bound of variable
VarMax = 3; % upper bound of variable

% PSO parameters
MaxIter = 150; % max number of iterations
nPop = 50; % population size
w = 1; % inertia
d = 0.99; % damping ratio of the inertia
c1 = 2; % acceleration coefficient 1
c2 = 2; % acceleration coefficient 2

% Initialization
x0.position = [];
x0.velocity = [];
x0.fitness = [];
x0.best.position = [];
x0.best.fitness = [];

x = repmat(x0, nPop, 1); % Make a population

global_best_fitness = inf;

% Generate initial population
for i = 1:nPop
    x(i).position = unifrnd(VarMin, VarMax, [1 nVar]); % generate random solutions
    x(i).velocity = zeros([1 nVar]); % initial velocity
    x(i).fitness = objective_function(x(i).position); % calculate the fitness
    x(i).best.position = x(i).position; % update the local best
    x(i).best.fitness = x(i).fitness; % update the local best
    if x(i).best.fitness < global_best_fitness
        global_best_fitness = x(i).best.fitness;
        global_best = x(i).best;
    end
end

B = zeros(MaxIter, 1); % Save the best fitness in each iteration

% Main program
for j = 1:MaxIter
    for i = 1:nPop
        x(i).velocity = w * x(i).velocity + c1 * rand([1 nVar]) .* (x(i).best.position - x(i).position) ...
            + c2 * rand([1 nVar]) .* (global_best.position - x(i).position); % update velocity
        x(i).position = x(i).position + x(i).velocity; % update position
        x(i).fitness = objective_function(x(i).position);
        if x(i).fitness < x(i).best.fitness
            x(i).best.position = x(i).position; % update the personal best
            x(i).best.fitness = x(i).fitness;
            if x(i).best.fitness < global_best.fitness
                global_best = x(i).best; % update the global best
            end
        end
    end
    w = w * d; % update the damping ratio
    % Save best fitness
    B(j) = global_best.fitness;
    disp(['Iteration ' num2str(j) ': Best fitness = ' num2str(B(j))]);
    plot(B(1:j, 1), 'r.'); drawnow
end