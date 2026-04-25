% =========================================================================
% Path Planning Algorithms Combined Script
% Requires: Peter Corke's Robotics Toolbox AND MATLAB Navigation Toolbox
% =========================================================================
clear; clc; close all;

%% 1. Map Setup and Initialization
disp('--- Initializing Map ---');
mapSize = 100;
map = zeros(mapSize, mapSize);

% Define obstacles 
map(20:50, 10:20) = 1; % Bottom-Left vertical
map(20:30, 20:30) = 1; % Bottom-Left horizontal
map(70:80, 20:50) = 1; % Top-Middle horizontal
map(40:70, 40:50) = 1; % Top-Middle vertical
map(10:20, 41:80) = 1; % Bottom-Right horizontal
map(22:80, 72:80) = 1; % Right vertical block 

% Define Start and Goal coordinates (x, y)
start_pos = [90, 10];
goal_pos = [30, 60];

% Visualize the base map
figure('Name', '1. Environment Map');
imagesc(map); colormap([1 1 1; 1 0 0]); 
hold on;
plot(start_pos(1), start_pos(2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 10); 
plot(goal_pos(1), goal_pos(2), 'bp', 'MarkerFaceColor', 'b', 'MarkerSize', 14); 
title('Environment Map'); legend('Start', 'Goal');
axis equal; axis([0 100 0 100]); set(gca, 'YDir', 'normal'); 

%% 2. Standard Dijkstra's Algorithm
disp('--- Running Standard Dijkstra ---');
figure('Name', '2. Standard Dijkstra');
[nrows, ncols] = size(map);
start_lin = sub2ind([nrows, ncols], start_pos(2), start_pos(1));
goal_lin = sub2ind([nrows, ncols], goal_pos(2), goal_pos(1));

dist = inf(nrows, ncols); dist(start_lin) = 0;
visited = false(nrows, ncols); parent = zeros(nrows, ncols);
[dx, dy] = meshgrid(-1:1, -1:1);

while ~visited(goal_lin)
    dist_unvisited = dist; dist_unvisited(visited) = inf;
    [min_dist, u] = min(dist_unvisited(:));
    
    if isinf(min_dist), break; end
    if u == goal_lin, break; end
    
    visited(u) = true;
    [uy, ux] = ind2sub([nrows, ncols], u);
    
    for k = 1:9
        if dx(k)==0 && dy(k)==0, continue; end
        nx = ux + dx(k); ny = uy + dy(k);
        
        if nx > 0 && nx <= ncols && ny > 0 && ny <= nrows
            if map(ny, nx) == 0 
                corner_cut = false;
                if (abs(dx(k)) == 1) && (abs(dy(k)) == 1)
                    if map(uy, nx) == 1 || map(ny, ux) == 1, corner_cut = true; end
                end
                
                if ~corner_cut
                    v = sub2ind([nrows, ncols], ny, nx);
                    if ~visited(v)
                        % Normal step cost based purely on distance (no wall penalties)
                        step_cost = norm([dx(k), dy(k)]); 
                        
                        if dist(u) + step_cost < dist(v)
                            dist(v) = dist(u) + step_cost; 
                            parent(v) = u;
                        end
                    end
                end
            end
        end
    end
end

% Reconstruct Path
path_dijkstra = []; curr = goal_lin;
if parent(curr) ~= 0 || curr == start_lin
    while curr ~= 0
        [cy, cx] = ind2sub([nrows, ncols], curr);
        path_dijkstra = [cx, cy; path_dijkstra]; curr = parent(curr);
    end
end

imagesc(map); colormap([1 1 1; 1 0 0]); hold on;
axis equal; axis([0 100 0 100]); set(gca, 'YDir', 'normal');
plot(path_dijkstra(:,1), path_dijkstra(:,2), 'k-', 'LineWidth', 2.5);
plot(start_pos(1), start_pos(2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 10);
plot(goal_pos(1), goal_pos(2), 'bp', 'MarkerFaceColor', 'b', 'MarkerSize', 14);
title('Path Planning: Standard Dijkstra');

%% 3. D* Algorithm 
disp('--- Running D* ---');
figure('Name', '3. D* Algorithm');
ds = Dstar(map);
ds.plan(goal_pos); 
path_dstar = ds.query(start_pos); 
ds.plot(path_dstar);
title('Path Planning: D* Algorithm');

%% 4. Probabilistic Roadmap Method (PRM)
disp('--- Running PRM ---');
figure('Name', '4. PRM');
prm = PRM(map);
prm.npoints = 500; 
prm.plan();
path_prm = prm.query(start_pos, goal_pos);

prm.plot(); 
hold on;
plot(path_prm(:,1), path_prm(:,2), 'g-', 'LineWidth', 6); % Thick Green Line
plot(start_pos(1), start_pos(2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 10);
plot(goal_pos(1), goal_pos(2), 'bp', 'MarkerFaceColor', 'b', 'MarkerSize', 14);
title('Path Planning: Probabilistic Roadmap (PRM)');

%% 5. Rapidly-exploring Random Tree (RRT) - Native MATLAB Version
disp('--- Running Official MATLAB plannerRRT ---');
figure('Name', '5. MATLAB RRT');

% 1. Create a native MATLAB Occupancy Map (Flipped to match earlier YDir normal maps)
occMap = binaryOccupancyMap(flipud(map));

% 2. Define the State Space (SE2 means 2D space [x, y, theta])
ss = stateSpaceSE2;
ss.StateBounds = [0 100; 0 100; -pi pi];

% 3. Create the State Validator for collision checking
sv = validatorOccupancyMap(ss);
sv.Map = occMap;
sv.ValidationDistance = 0.5; 

% 4. Initialize the RRT Planner
planner = plannerRRT(ss, sv);
planner.MaxConnectionDistance = 5; 
planner.MaxIterations = 5000;

% 5. Define Start and Goal [x, y, theta]
start_state = [start_pos(1), start_pos(2), 0];
goal_state  = [goal_pos(1), goal_pos(2), 0];

% 6. Plan the Path
disp('Growing the native RRT tree...');
[pthObj, solnInfo] = planner.plan(start_state, goal_state);

% 7. Visualize High-Contrast Results
% Black obstacles ([0 0 0]) so the red path and cyan tree show up clearly
imagesc(map); colormap([1 1 1; 0 0 0]); 
hold on;
axis equal; axis([0 100 0 100]); set(gca, 'YDir', 'normal'); 

% --- Adaptive Tree Plotting ---
tree = solnInfo.TreeData;
if size(tree, 2) >= 4
    % If your MATLAB version includes parent IDs, draw the branches
    for i = 1:size(tree, 1)
        parentIdx = tree(i, 4); 
        if isnumeric(parentIdx) && round(parentIdx) == parentIdx && parentIdx > 0
            plot([tree(i, 1), tree(parentIdx, 1)], [tree(i, 2), tree(parentIdx, 2)], 'Color', [0 0.6 0.8], 'LineWidth', 0.5);
        end
    end
else
    % If your MATLAB version only outputs [x, y, theta], draw the searched nodes as dots
    plot(tree(:, 1), tree(:, 2), '.', 'Color', [0 0.6 0.8], 'MarkerSize', 4);
end

% Plot the final planned path (Thick Red)
plot(pthObj.States(:,1), pthObj.States(:,2), 'r-', 'LineWidth', 3.5);

% Plot Start (Green) and Goal (Red) points
plot(start_pos(1), start_pos(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
plot(goal_pos(1), goal_pos(2), 'rp', 'MarkerFaceColor', 'r', 'MarkerSize', 14);

title('Path Planning: High-Contrast MATLAB plannerRRT');
hold off;

disp('--- All planning complete! ---');