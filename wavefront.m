%% Autonomous Robot 
% Potential Function - Wavefront Planner
% Author : Pramita Winata 
% Master VIBOT
%%

function [ new_map, trajectory ] = wavefront( map, start_row, start_column )
%% This function will construct the new weighted map, compute the trajectory and plot the map


global VALUE_GOAL;
VALUE_GOAL = 2;

trajectory = [];
[x_goal, y_goal] = find_goal(map, VALUE_GOAL);
% check if the start_row and start_column are inside the map
if( (start_row < 1 ) || (start_row > size(map,1)) ||...
    (start_column < 1 ) || (start_column > size(map,2)))
   error(['Start point must be inside the map. Please specify a point within [ ',num2str(size(map,1)),'..',num2str(size(map,2)) ,' ]']);
end

goal_point = [x_goal, y_goal ];

new_map = build_new_map( map, goal_point );

start_point = new_map(start_row, start_column);

% check if the environment has solution and the start point is valid
if ( start_point == 0 ) || ( start_point == 1 )  
    disp('NO solution.')
else 
  trajectory = compute_trajectory( new_map, start_row, start_column );
end

plot_map(map, trajectory );
end

function  [x_goal, y_goal] = find_goal(map, value_goal)
%% This function will find the goal point in the map
% It will display error messages if the map is invalid
[x_goal, y_goal] = find(map == value_goal);

if isempty(x_goal)  || isempty(y_goal)
    error(['Goal cannot be found. Please specify a point with value ', num2str(value_goal),' in the map']);
elseif size(x_goal,1)> 1 || size(y_goal,1)> 1
    error(['There are multiple goals. Please specify only one point with value ', num2str(value_goal),' in the map']);
else
   x_goal = x_goal(1);
   y_goal = y_goal(1);
end
end

function map = build_new_map(map, goal_point )
%% This function will construct a new weighted map based on the 8-connected points

[i_height,i_width  ] = size(map);
i_size = i_height*i_width;

flag = zeros(i_height,i_width );

queue = zeros( i_size, 2);
queue_front = 1;
queue_last = 2 ;
queue(1,:) = goal_point;

while (  queue_front < queue_last )
    %&& ~goal_found ) 
    i = queue(queue_front, 1);
    j = queue(queue_front, 2);
    neighboors = get_neighboors(i,j);
    current_label = map(i,j);
    for n = 1 : length(neighboors)
        n_x =  neighboors( n,1 ) ; n_y = neighboors(n,2 );
 
        if (  n_x > 1 && n_y>1....
              && n_x <= i_height-1 && n_y <= i_width-1....
              && map( n_x,  n_y ) == 0....
              && flag(n_x,n_y) == 0 )
          
            
            map( n_x, n_y ) = current_label + 1;

            flag(n_x,n_y) = 1;
                
            queue( queue_last, : ) =  [n_x,n_y] ;
            queue_last = queue_last + 1; 

        end   
    end
    queue_front = queue_front + 1; 
end

end

function neighbors = get_neighboors(r, c)
%% Function to get the neighbors of a given pixel
%4-th connectivity(up, right, down, left ) will be prioritzed

neighbors = [r-1, c;                   % up
             r,   c+1;                 % right 
             r+1, c;                   % down
             r,   c-1;                 % left
             r-1, c-1;                 % left-up
             r-1, c+1;                 % right-up
             r+1, c+1;                 % right-down 
             r+1, c-1];                % left-down            
end

function trajectory = compute_trajectory( new_map, start_row, start_column)
%% This function will compute the trajectory to reach the goal

[i_height, i_width] = size(new_map);
value = new_map(start_row, start_column);
trajectory(1,:) = [start_row, start_column];
global VALUE_GOAL;
counter_trajectory = 1;

while( value ~= VALUE_GOAL )
    row = trajectory(counter_trajectory,1);
    column = trajectory(counter_trajectory,2);   
    neighbors = get_neighboors(row, column);
   
    
    for i = 1 : size(neighbors)        
        n_x =  neighbors( i,1 ) ; n_y = neighbors(i,2 );
        % check that neighbors are valid
         if ( n_x >= 1 && n_y>=1 && n_x <= i_height && n_y <= i_width )
            neighbors_values(:,i) = new_map(neighbors(i,1), neighbors(i,2));
         else
            neighbors_values(:,i) = 1;
         end
    end
    % gradient descendt
    difference = value - neighbors_values;
    evaluate = difference .* (neighbors_values(:) ~= 1)';
    index = find( evaluate(:) == max(evaluate(:)), 1, 'first');
          
    % update values for next loop
    counter_trajectory = counter_trajectory + 1;
    row = neighbors(index,1);
    column  = neighbors(index,2);
    trajectory(counter_trajectory,:) = [row, column];
    
    value = new_map(row, column);
end

end

function plot_map(map, trajectory )
 %% This function will plot the given map and trajectory
 % Start point will be red
 % Goal point will be green
   figure
   imagesc(map);
   colormap(cool);
   hold on

   plot(trajectory(:,2), trajectory(:,1),'m--', 'LineWidth', 2)
   plot(trajectory(1,2), trajectory(1,1),'rs','MarkerFaceColor','r','MarkerSize',10);
   plot(trajectory(end,2), trajectory(end,1),'gd','MarkerFaceColor','g','MarkerSize',10);
    
    
end