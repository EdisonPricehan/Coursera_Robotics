% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% % the number of grids for 1 meter.
myResol = param.resol;
% % the initial map size in pixels
myMap = zeros(param.size);
% % the origin of the map in pixels
myorigin = param.origin; 
% 
% % 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(pose,2); % number of time steps
n = size(scanAngles,1); % number of emitted rays at certain time step
for i = 1:N % for each time,
    pos_robot = repmat([pose(1,i) pose(2,i)]',1,n); % x and y values of robot
    theta = pose(3,i) * ones(n,1); % theta of robot
    % construct real location of occupied point in global coord system after rotation and
    % translation
    real_occ_x = ranges(:,i) .* cos(scanAngles + theta);
    real_occ_y = -ranges(:,i) .* sin(scanAngles + theta);
    real_occ = [real_occ_x'; 
                real_occ_y']; 
    real_occ = real_occ + pos_robot;
    % construct grid 
    grid_occ = ceil(myResol * real_occ) + repmat(myorigin,1,n);
    grid_occ = unique(grid_occ','rows'); % filter out the repetitive grid cells
    grid_occ_id = sub2ind(size(myMap), grid_occ(:,2), grid_occ(:,1)); % convert to linear indices 
    % construct free cells by bresenham function
    grid_free_id = [];
    for j = 1:size(grid_occ,1)
        [free_x, free_y] = bresenham(ceil(pose(1,i)*myResol)+myorigin(1),ceil(pose(2,i)*myResol)+myorigin(2),grid_occ(j,1),grid_occ(j,2));
        free_id = sub2ind(size(myMap), free_y, free_x);
        grid_free_id = [grid_free_id; free_id]; % append free cells obtained from each obstructed ray
    end
    grid_free_id = unique(grid_free_id); % filter out repeated grid free cells
    % update metric values of occupied and free grid cells 
    myMap(grid_occ_id) = myMap(grid_occ_id) + lo_occ;
    myMap(grid_free_id) = myMap(grid_free_id) - lo_free;
% Limit the log-odds in given values   
myMap(myMap > lo_max) = lo_max;
myMap(myMap < lo_min) = lo_min;


% Find grids hit by the rays (in the gird map coordinate)

% Find occupied-measurement cells and free-measurement cells

% Update the log-odds

% Saturate the log-odd values

% Visualize the map as needed

end

end

