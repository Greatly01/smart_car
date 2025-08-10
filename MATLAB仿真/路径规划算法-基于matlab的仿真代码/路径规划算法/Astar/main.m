%%age/k03936lvfuq.html Define a small map
map = false(10);

% Add an obstacle
map (1:5, 6) = true;
map (3, 1:3) = true;
map (8, 3:8) = true;
map (2, 8:10) = true;
map (4:7, 9) = true;
start_coords = [10, 1];
dest_coords  = [1, 10];
%%
close all;
%  [route, numExpanded] = DijkstraGrid (map, start_coords, dest_coords);
 % Uncomment following line to run Astar

 [route, numExpanded] = AStarGrid (map, start_coords, dest_coords);
%HINT: With default start and destination coordinates defined above, numExpanded for Dijkstras should be 76, numExpanded for Astar should be 23.
