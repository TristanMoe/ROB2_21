% Create Occupancy Map 
image = imread('shannon.png');

% Convert to grayscale and black and white image
grayimage = rgb2gray(image);
bwimage = grayimage < 220;

grid = robotics.BinaryOccupancyGrid(bwimage);
map = flipud(grid.getOccupancy());

%% Create Shortest Path

start = [1111; 600]; 
goal = [150; 400]; 

dx = DXform(map);
dx.plan(goal);
path = dx.query(start, 'animate'); 
