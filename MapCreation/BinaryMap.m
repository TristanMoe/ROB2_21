% Create Occupancy Map 
image = imread('shannon.png');

% Convert to grayscale and black and white image
grayimage = rgb2gray(image);
bwimage = grayimage < 220;

grid = robotics.BinaryOccupancyGrid(bwimage);
show(grid)

%% Create Shortest Path 
start = [140 200]; 
goal = [0 0]; 

dx = DXform(grid);
dx.plan(goal);
path = dx.query(start, 'animate'); 
