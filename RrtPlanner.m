%% RrtPlanner
classdef RrtPlanner < handle
%
% *Description:* MAIN FUNCTION: This function is a basic implementation of
% the multi-tree Rapidly-exploring Random Tree (RRT) search algorithm with
% the ability to  place discretely possitioned seeds throughout the
% environment which may of may not take root as trees
%
% *Authors:* Gavin Paul & Matthew Clifton
%
% *Last Updated:* 29th November 2012
%
% *Features:*
%   - Multiple trees
%   - Discrete seeding environmental coverage
%   - 2D or 3D search space
%   - Obstacle avoidance
%   - Auto-connect to goal
%   - Path smoothing
%
% *To do:*
%  - Adaptive sampling
%  - Approximate nearest neighbour search
%% Usage
% treesMax = 28; %How many multiple trees (must be at least 2, 1 for source and 1 for destination
% seedsPerAxis = 3; %Number of seeds allowed on each axis (discretely placed seeds which idealy helps the RRT expansion)
% wallCount = 5; %Number of mock walls to be placed in the environment
% rrt = RrtPlanner(treesMax,seedsPerAxis,wallCount)
% rrt.SetStart([0 -0.9 0]);
% rrt.SetGoal([0 +0.9 0]);
% plot3(rrt.smoothedPath(:,1),rrt.smoothedPath(:,2),rrt.smoothedPath(:,3),'k*');
% rrt.Run()
% delete(rrt);
%
% obstacleFilename = 'obstacles.txt';
% seedsPerAxis = 7;
% treesMax = seedsPerAxis^3*3+2;
% rrt = RrtPlanner(treesMax,seedsPerAxis,obstacleFilename);
% rrt.drawingSkipsPerDrawing = 30;
% rrt = RrtPlanner(treesMax,seedsPerAxis,obstacleFilename);
% rrt.Run()
% plot3(rrt.path(:,1),rrt.path(:,2),rrt.path(:,3),'k*');
    
    properties
        %% RUN OPTIONS
        %  ************************************************************************
        % Display                           => Activate visual display, SLIGHTLY SLOWER
        doDraw=true; 
        % Skip this number of drawings before doing an update to the figure (speeds up plotting)
        drawingSkipsPerDrawing = 10;
        
        % Obstacle Avoidance                => Include obstacles at all
        avoidObstacles=true;        
        % Path Smoothing                    => Shorten final path
        smooth_path=true;
       
        % Number of walls in the environment
        wallCount = 3;
        
        % Number of seeds
        seedsPerAxis = 3;
        %% VARIABLES
        %  ************************************************************************
        % Iterations                        => Maximum number of search iterations
        maxIterations=2000;
        % Maximum number of search trees    => Maximum number of search iterations
        treesMax = 30;
        % Search Space Limits               => [x(min) x(max) y(min) y(max) z(min) z(max)]
        % lim=[-0.5 +0.5;-0.5 +0.5 ;-0.5 +0.5];
        lim=[-15 +15;-15 +15 ;0 +8];
        % Default Start and Goal  => Point [x y z]
        start=[-13.00 -12 +0.5];
        goal =[+14 +7 +4.00];
        % Number of attempts at path smoothing
        nsmooth=1000;
        
        % Figure handle
        figure_h = [];
        
        % Axis handle
        axis_h = [];
        
        % The main data structure
        rrt;
        
        % The obstacles
        obs;
        
        % The plane parameters of each obstacle plane
        obstaclePlaneParameters;
        
        % The number of obstacles
        obstacleCount;
        
        % The number of collision checks
        collisionCheckCount = 0;
                
        % The number of drawing that have been skipped since the last draw command
        skippedDrawings = 0;
        
        % The number of times nearest neighbour is called
        nearestNeighbourCount = 0;
        
        % The handles for the lines and points in the figure
        plotHandles = [];
        
        % The path from start to goal
        path = [];
        
        % Smoothed path
        smoothedPath = [];
                
    end
    
    properties (SetAccess=protected)
        % Obstacle definitions              => Use objects.txt object definition or default
        obstacleFilename = '';
        
        % The plot handles for the obstacles
        obsPlot_h
        
        % Plot handles for start node
        startNodePlot_h
        
        % Plot handles for end node
        goalNodePlot_h
        
        % Plot handle for the smoothed green line
        smoothedPathPlot_h
    end
    
    methods
%% .. structors        
% *Inputs:* 
% treesMax = How many multiple trees (must be at least 2, 1 for source and 1 for destination
% seedsPerAxis = Number of seeds allowed on each axis (discretely placed seeds which idealy helps the RRT expansion)
% wallCount = the Number of mock walls to be placed in the environment
        function self = RrtPlanner(treesMax,seedsPerAxis,wallCountOrObstacleFilename)
            % Check inputs
            if 1 <= nargin
                self.treesMax = treesMax;                
                if 2 <= nargin
                    self.seedsPerAxis = seedsPerAxis;
                    if 3 <= nargin && isa(wallCountOrObstacleFilename,'char')
                        self.wallCount = 0;
                        self.obstacleFilename = wallCountOrObstacleFilename;
                    elseif 3 <= nargin && isa(wallCountOrObstacleFilename,'numeric')
                        self.obstacleFilename = '';
                        self.wallCount = wallCountOrObstacleFilename;                       
                    end
                end
            end
            % Define or add obstacles
            self.GenerateObstacles();
        end
        
        
%% Run
% Main RRT search algorithm
        function Run(self)
            % Initial plotting and environment setup
            self.InitDisplay();
            
            tic;
            % Continue search while the number of steps is less than Iterations and
            % treesMax, and a path has not been found
            for cur_it = 1:self.maxIterations
                % GENERATE A NEW POINT
                new_pnt = self.NewPoint(); %if doDraw; plot3(new_pnt(1),new_pnt(2),new_pnt(3),'.c'); end
                % FIND NEAREST NEIGHBOURS    
                [d2nodes,d2edges] = NearestNeighbour(new_pnt,self.rrt);
                self.nearestNeighbourCount = self.nearestNeighbourCount + 1;
                % CONNECT TO NEAREST NEIGHBOUR
                objective = self.Connect(d2nodes,d2edges,new_pnt);
                % DRAW
                if self.doDraw && self.drawingSkipsPerDrawing <= self.skippedDrawings
                    self.DisplayTree(cur_it);
                    self.skippedDrawings = 0;
                else
                    self.skippedDrawings = self.skippedDrawings + 1;
                end
            
                % Check if we can break out yet
                if objective; break; end    
            end
            toc;
            % PATH OUTPUT
            self.TracePath(objective);
            % PATH SMOOTHING
            if self.smooth_path && objective
                display('Doing path smoothing now');
                tic
                self.SmoothPath();
                toc;
            end
        end
        
%% SetObstacleFilename
        function SetObstacleFilename(self,value)
            if isa(value,'char')
                self.obstacleFilename = value;
                self.GenerateObstacles();
            end
        end
%% SetUpDataStructures
        function SetUpDataStructures(self)
            self.rrt = [];
            % max trees (mt) needs to be at least 2 trees 1 start and 1 dest
            if self.treesMax < 2
                warning('You must have at least 2 trees, 1 for start and 1 for dest'); %#ok<WNTAG>
                self.treesMax = 2;
            end
            
            % seedsPerAxis^3 + 2 must be less than or equal to the max trees since they are per axis
            while self.treesMax < self.seedsPerAxis^3+2
                self.seedsPerAxis = self.seedsPerAxis-1;
                display(['Reducing num seeds on each dimension to ',num2str(self.seedsPerAxis)]);
            end
            
            % Inialise RRT data structures                        
            self.rrt = [];
            % Setup the first tree and first node to be the start and finish
            self.SetStart(self.start);
            self.SetGoal(self.goal);
            % Plant the seeds discretely in the environment
            seed_cords = zeros(self.seedsPerAxis^3,3);
            seedCounter = 0;
            for i=1:self.seedsPerAxis
                for j=1:self.seedsPerAxis
                    for k=1:self.seedsPerAxis
                        potentialSeed = [self.lim(1,1) + i*(self.lim(1,2)-self.lim(1,1))/(self.seedsPerAxis+1) ...
                                        ,self.lim(2,1) + j*(self.lim(2,2)-self.lim(2,1))/(self.seedsPerAxis+1) ...
                                        ,self.lim(3,1) + k*(self.lim(3,2)-self.lim(3,1))/(self.seedsPerAxis+1)];
                        if ~self.IsOnObstaclePlane(potentialSeed)
                            seedCounter = seedCounter + 1;
                            seed_cords(seedCounter,:) = potentialSeed;                            
                        end                        
                    end
                end
            end
            seed_cords = seed_cords(1:seedCounter,:);
            
            % Plant seeds a RRT tree to either grow or joint with other trees
            self.PlantNewSeeds(seed_cords);
            
            % Initialise the remaining rrt nodes up to the max number of trees
            for i = size(seed_cords,1) + 3:self.treesMax
                self.rrt(i).valid = 0;
                self.rrt(i).cords = [0,0,0];
                self.rrt(i).parent = 0;
            end    
        end
                
        function delete(self)
            try delete(self.figure_h);end %#ok<TRYNC>
        end
        
%% SetStart
% Set the start position        
        function SetStart(self,start)
            if self.IsOnObstaclePlane(start)
                error('The start is on an obstacle. Please change the start or the environment');
            end
                
            self.start = start;
            self.PlantNewSeeds(self.start);
        end
%% SetGoal
% Set the goal position                
        function SetGoal(self,goal)
            if self.IsOnObstaclePlane(goal)
                error('The goal is on an obstacle. Please change the start or the environment');
            end
            self.goal = goal;
            self.PlantNewSeeds(self.goal);
        end
%% PlantNewSeeds
% Plant another seed and start another rrt off        
        function PlantNewSeeds(self,point)
            for i = 1:size(point,1)
                if self.IsOnObstaclePlane(point(i,:))
                    warning(['The point ',point(i,:),' is on an obstacle plane so ignoring it']); %#ok<WNTAG>
                end
                rrtTreeCount = size(self.rrt,2);
                self.rrt(rrtTreeCount+1).valid = 1;
                self.rrt(rrtTreeCount+1).cords = point(i,:);
                self.rrt(rrtTreeCount+1).parent = 0;
            end
        end
        
%% InitDisplay
% *Description:* Plots and outputs some info
        function InitDisplay(self)
            if ~self.doDraw
                return;
            end
            % Output to command window
            fprintf('\n******************************************\n');
            fprintf('***   Rapidly-Exploring Random Trees   ***\n');
            fprintf('******************************************\n\n');
            fprintf('Max. number of steps: %d \n',self.maxIterations);
            fprintf('Max. number of trees: %d \n\n',self.treesMax);
            % Output to figure
            try self.figure_h = figure(self.figure_h);
            catch  %#ok<CTCH>
                self.figure_h = figure;
            end
            title('Rapidly-Exploring Random Trees (Step 1)');
            set(self.GetAxisHandle(),'xtick',[],'ytick',[],'ztick',[]);
            %axis off;
            axis([self.lim(1,1), self.lim(1,2) ...
                 ,self.lim(2,1), self.lim(2,2) ...
                 ,self.lim(3,1), self.lim(3,2)],'square');
            hold on;
            % Plot initial node
            try delete(self.startNodePlot_h);end %#ok<TRYNC>
            self.startNodePlot_h = plot3(self.start(1),self.start(2),self.start(3),'marker','.','color','k','Parent',self.GetAxisHandle());
            % Plot goal node
            try delete(self.goalNodePlot_h);end %#ok<TRYNC>
            self.goalNodePlot_h = plot3(self.goal(1),self.goal(2),self.goal(3),'marker','.','color','b','Parent',self.GetAxisHandle());
            % Plot obstacles
            for i = 1:length(self.obsPlot_h)
                try delete(self.obsPlot_h(i));end %#ok<TRYNC>
            end
            self.obsPlot_h = [];
            if size(self.obs,1)>0
                for i = 1:size(self.obs,3)                    
                    self.obsPlot_h(i) = fill3([self.obs(1,1,i) self.obs(2,1,i) self.obs(3,1,i) self.obs(4,1,i) self.obs(1,1,i)] ...
                                             ,[self.obs(1,2,i) self.obs(2,2,i) self.obs(3,2,i) self.obs(4,2,i) self.obs(1,2,i)] ...
                                             ,[self.obs(1,3,i) self.obs(2,3,i) self.obs(3,3,i) self.obs(4,3,i) self.obs(1,3,i)] ...
                                             ,'b','EdgeAlpha',0,'Parent',self.GetAxisHandle());
                    alpha(0.1);
                end
            end
            
            % Delete all rrt lines and previous paths
            for i = 1:length(self.plotHandles)
                try delete(self.plotHandles(i).lines);end; %#ok<TRYNC>
            end
            
            % Try and delete the smoothed path line
            try delete(self.smoothedPathPlot_h);end; %#ok<TRYNC>            
        end        
%% SetAvoidObstacles
        function SetAvoidObstacles(self,value)
            if nargin == 1 || value
                self.avoidObstacles = true;                
            elseif ~value
                self.avoidObstacles = false;
            end
            self.GenerateObstacles();
        end
            
%% Function GenerateObstacles
% *Description:* This either loads the obstacleFilename which is a setup as
% multiple sets of 4 3D point which define a planar obstacle. Or we use the
% default walls where wallCount decides how many walls there are to be
        function GenerateObstacles(self)
            self.obs = [];
            self.obstacleCount = 0;
            self.obstaclePlaneParameters = [];
                
            if ~self.avoidObstacles                
                return;
            end            
            
            % Obstacles                     => Define a obstacles using four points [x(1:4,:) y(1:4,:) z(1:4,:)]
            if ~isempty(self.obstacleFilename)
                try obs_temp=load(self.obstacleFilename);
                catch  %#ok<CTCH>
                    error(['Cant load ',self.obstacleFilename]);
                end    
                num_obs = size(obs_temp,1)/4;
                for i=1:num_obs
                    self.obs(:,:,i)=obs_temp(i*4-3:i*4,:);
                end
            else % Add some number of obstacle walls
                if self.wallCount == 1
                    y = 0;
                    self.obs(:,:,1:4) = self.WallAtY(y,1);
                elseif 1 < self.wallCount
                    for i=1:self.wallCount
                        y = self.lim(2,1) + 0.2 + (i-1)*(self.lim(2,2)-self.lim(2,1)-0.4)/(self.wallCount-1);
                        if mod(i,2) == 1
                            self.obs(:,:,i*4-3:i*4) = self.WallAtY(y,1);
                        else
                            self.obs(:,:,i*4-3:i*4) = self.WallAtY(y,2);
                        end
                    end
                end
            end
            
            % Precalculate the 4 obstacle plane parameters [a,b,c,d]
            self.obstaclePlaneParameters = zeros(size(self.obs,3),4);
            for i = 1:size(self.obs,3)
                % Normal vector (this code is quicker than 'cross(obs2-obs1,obs1-obs3)')
                v1 = (self.obs(2,:,i) - self.obs(1,:,i))'; 
                v2 = (self.obs(1,:,i) - self.obs(3,:,i))';
                normalVec = [v1(2,:).*v2(3,:) - v1(3,:).*v2(2,:) ...
                            ;v1(3,:).*v2(1,:) - v1(1,:).*v2(3,:) ...
                            ;v1(1,:).*v2(2,:) - v1(2,:).*v2(1,:)]';
                % Plane equation
                self.obstaclePlaneParameters(i,:) = [normalVec(1:3), -sum(self.obs(1,:,i).*normalVec)];
                self.obstacleCount = size(self.obstaclePlaneParameters,1);
            end           
            
            % Need to clear the data structure since obstacles may have changed
            self.SetUpDataStructures();
            
            % Also Reinitialise the display
            self.InitDisplay();            
        end
        
%% Function Connect
% 
% *Description:* This function attempts to connect new_pnt to the nearest node or edge of 
% each tree.
        function objective = Connect(self,d2nodes,d2edges,new_pnt)
            objective=0;
            addedtotree=zeros([size(self.rrt,2),1]);
            
            % Create local variable. Could cause problems if self.rrt is used in the mean time
            rrtLocal = self.rrt;
            
            %find the valid trees
            for t=1:size(rrtLocal,2)
                if ~rrtLocal(t).valid
                    continue;
                end
                %get the min values of the nodes and edges
                [~,minNode_index]=min(d2nodes(t).vals);
                [~,minEdge_index]=min(d2edges(t).vals);
                %determine the closest (node or edge)
                if d2nodes(t).vals(minNode_index)<=d2edges(t).vals(minEdge_index) %then try and get to the node first
                    % Check for collision  
                    if ~self.CollisionCheck(new_pnt,rrtLocal(t).cords(minNode_index,:));
                        rrtLocal(t).parent=[rrtLocal(t).parent;minNode_index];
                        rrtLocal(t).cords=[rrtLocal(t).cords;new_pnt];
                        addedtotree(t)=1;
                    end
                else %try and get to the edge
                    % Check for collision 
                    if d2edges(t).vals(minEdge_index)<inf && ~self.CollisionCheck(new_pnt,d2edges(t).node(minEdge_index,:));
                        %this is really tricky!! because the index
                        %of the edges where the first point is the start of
                        %the line, this will also be the parent of that
                        %middle node
                        rrtLocal(t).parent=[rrtLocal(t).parent;rrtLocal(t).parent(minEdge_index+1)];
                        rrtLocal(t).cords=[rrtLocal(t).cords;d2edges(t).node(minEdge_index,:)];
                        rrtLocal(t).parent=[rrtLocal(t).parent;size(rrtLocal(t).parent,1)];
                        rrtLocal(t).cords=[rrtLocal(t).cords;new_pnt];
                        addedtotree(t)=1;
                    end
                end        
            end
            % need to find out if we need to add a tree and put the new point as the
            if sum(addedtotree)>1 %then we have connected 2 trees
                trees_added_to=find(addedtotree==1);  
                %the linking node from first trees parent
                parentnode=rrtLocal(trees_added_to(1)).parent(end);
                    %this bit is tricky TOO since you have to plant the tree by one of
                    %its branches and then somehow make the parent correct. The newpoint added to 2
                    %different trees currently has 2 (or more) parents
                %go through all other tree which were connected in
                for i=2:size(trees_added_to)
                    %store the original parents
                    original_parents=rrtLocal(trees_added_to(i)).parent;
                    %start at the connecting node (which should be the last node)
                    curnode=size(rrtLocal(trees_added_to(i)).parent,1);
                    %asign new parent to the other trees connecting node
                    rrtLocal(trees_added_to(i)).parent(curnode)=parentnode;
                    %go through and trace route back to start and swap parents around,
                    %all nodes not on this route keep the same parents (shifted)
                    while original_parents(curnode)~=0
                        thisnodes_parent=original_parents(curnode);
                        rrtLocal(trees_added_to(i)).parent(thisnodes_parent)=curnode;
                        curnode=thisnodes_parent;
                    end
                    %find the nodes currently connected to the parent node in the first tree
                    connectedToLastNode = rrtLocal(trees_added_to(1)).parent==size(rrtLocal(trees_added_to(1)).parent,1);
                    %concaternate coords into first tree
                    rrtLocal(trees_added_to(1)).cords=[rrtLocal(trees_added_to(1)).cords(1:end-1,:);rrtLocal(trees_added_to(i)).cords];
                    %concaternate parents together and shift the seconds ones
                    rrtLocal(trees_added_to(1)).parent=[rrtLocal(trees_added_to(1)).parent(1:end-1);...
                                                   rrtLocal(trees_added_to(i)).parent(1:end-1)+size(rrtLocal(trees_added_to(1)).parent,1)-1;...
                                                   parentnode];
                    %update the nodes connected to the parent node in the first tree to the new parent node position
                    rrtLocal(trees_added_to(1)).parent(connectedToLastNode) = size(rrtLocal(trees_added_to(1)).parent,1);
                    rrtLocal(trees_added_to(i)).valid=0;
                end
                %since we have connected 2 trees we may now have a solution
                %test to see if start and goal are in the same tree
                foundstart=find(rrtLocal(trees_added_to(1)).cords(:,1) == self.start(1) ...
                              & rrtLocal(trees_added_to(1)).cords(:,2) == self.start(2) ...
                              & rrtLocal(trees_added_to(1)).cords(:,3) == self.start(3),1);
                foundgoal = find(rrtLocal(trees_added_to(1)).cords(:,1) == self.goal(1) ...
                               & rrtLocal(trees_added_to(1)).cords(:,2) == self.goal(2) ...
                               & rrtLocal(trees_added_to(1)).cords(:,3) == self.goal(3),1);
                %then the solution has been found
                if ~isempty(foundstart) && ~isempty(foundgoal)
                    objective = 1;
                end
            %if we have not added any then we try and make a new tree if we can, otherwise point is lost    
            elseif sum(addedtotree)==0
                %couldn't connect to any points or edges within the first tree
                %so make a new tree
                for i=1:size(rrtLocal,2)
                    if rrtLocal(i).valid==0
                        rrtLocal(i).valid=1;
                        rrtLocal(i).cords=new_pnt;
                        rrtLocal(i).parent=0;
                        break
                    end
                end
            end
            
            % Set back to be rrt. Could cause problems if self.rrt is used in the mean time
            self.rrt = rrtLocal;
        end
%% Function NewPoint
%
% *Description:* This function randomly samples the search space. and produces a new node
        function newPoint = NewPoint(self)
            % Size of search space
            range = abs(self.lim(:,2)-self.lim(:,1));
            % Randomly generate point not on an obstacle
            newPoint = [self.lim(1,1) + range(1)*rand ...
                       ,self.lim(2,1) + range(2)*rand ...
                       ,self.lim(3,1) + range(3)*rand];
            
            % If the new point is on an obstacle plane the get another one
            while self.IsOnObstaclePlane(newPoint)
                newPoint=[self.lim(1,1) + range(1)*rand ...
                         ,self.lim(2,1) + range(2)*rand ...
                         ,self.lim(3,1) + range(3)*rand];
            end            
        end       
        
        
%% Check if the point is on any of the obstacle planes
        function result = IsOnObstaclePlane(self,point)
            if ~self.avoidObstacles
                result = false;
                return;
            end
                
            %Check if the point is on any of the planes. 
            % This happens if the point [x,y,z] returns true to the following inequality
            % ax + by + cz == -d
            result = ~isempty(find(sum(self.obstaclePlaneParameters(:,1:3) ...
                                  .* repmat(point,self.obstacleCount,1),2) ...
                                  == -self.obstaclePlaneParameters(:,4) ...
                              ,1));
        end
                
        
%% Function TracePath
%
% *Description:* This function traces the path back through the tree.
        function TracePath(self,objective)
            % Show final result
            if objective
                if self.doDraw; fprintf('Found a path to the goal!\n\n'); end;
                %it will be in the first tree
                goalnode = find(self.rrt(1).cords(:,1) == self.goal(1) ...
                               &self.rrt(1).cords(:,2) == self.goal(2) ...
                               &self.rrt(1).cords(:,3) == self.goal(3));
                % Trace path back through tree
                self.path = self.goal;
            %     parent=n;
                while self.rrt(1).parent(goalnode)~=0
                    parent = self.rrt(1).parent(goalnode);
                    self.path = [self.rrt(1).cords(parent,:);self.path];
                    goalnode = parent;
                end
                % Plot path
                if self.doDraw; 
                    %delete all other crap off screen
                    for t=1:size(self.rrt,2)
                       for i=2:size(self.rrt(t).parent,1)
                           try delete(self.plotHandles(t).lines(i));end  %#ok<TRYNC>
                       end
                       try delete(self.plotHandles(t).points);end  %#ok<TRYNC>
                    end
                    self.plotHandles(t).lines = plot3(self.path(:,1),self.path(:,2),self.path(:,3),'LineWidth',2,'Color','r','Parent',self.GetAxisHandle());                     
                end
            else
                if self.doDraw; fprintf('Failed to find a path to the goal.\n\n'); end;
                self.path = [];
            end   
        end
        
%% Function SmoothPath
%
% *Description:* Shortens a path by repeatedly attempting to connect two 
% randomly selected points along the path together.
        function SmoothPath(self)
            final_path = self.path;
            
            % Perform path smoothing
            for i=1:self.nsmooth
                % Randomly select two path segments
                path_length=size(final_path,1);
                p1=ceil((path_length-1)*rand);
                p2=ceil((path_length-1)*rand);
                while (p2==p1); p2=ceil((path_length-1)*rand); end;
                pt=p1; if p1>p2; p1=p2; p2=pt; end;
                % Randomly select two points from the two path segments
                r1 = rand;
                if final_path(p1,1)>final_path(p1+1,1); pnt1(1)=final_path(p1,1)-abs(final_path(p1,1)-final_path(p1+1,1))*r1; else pnt1(1)=final_path(p1,1)+abs(final_path(p1,1)-final_path(p1+1,1))*r1; end
                if final_path(p1,2)>final_path(p1+1,2); pnt1(2)=final_path(p1,2)-abs(final_path(p1,2)-final_path(p1+1,2))*r1; else pnt1(2)=final_path(p1,2)+abs(final_path(p1,2)-final_path(p1+1,2))*r1; end
                if final_path(p1,3)>final_path(p1+1,3); pnt1(3)=final_path(p1,3)-abs(final_path(p1,3)-final_path(p1+1,3))*r1; else pnt1(3)=final_path(p1,3)+abs(final_path(p1,3)-final_path(p1+1,3))*r1; end
                
                r2 = rand;
                if final_path(p2,1)>final_path(p2+1,1); pnt2(1)=final_path(p2,1)-abs(final_path(p2,1)-final_path(p2+1,1))*r2; else pnt2(1)=final_path(p2,1)+abs(final_path(p2,1)-final_path(p2+1,1))*r2; end
                if final_path(p2,2)>final_path(p2+1,2); pnt2(2)=final_path(p2,2)-abs(final_path(p2,2)-final_path(p2+1,2))*r2; else pnt2(2)=final_path(p2,2)+abs(final_path(p2,2)-final_path(p2+1,2))*r2; end
                if final_path(p2,3)>final_path(p2+1,3); pnt2(3)=final_path(p2,3)-abs(final_path(p2,3)-final_path(p2+1,3))*r2; else pnt2(3)=final_path(p2,3)+abs(final_path(p2,3)-final_path(p2+1,3))*r2; end
                % Connect the two points
                if ~self.CollisionCheck(pnt1,pnt2);
                    % Update Path
                    path1=final_path(1:p1,:);
                    path2=[pnt1;pnt2];
                    path3=final_path(p2+1:end,:);
                    final_path=[path1;path2;path3];
                end
            end
            
            self.smoothedPath = final_path;
            % Plot final path
            if self.doDraw
                self.smoothedPathPlot_h = plot3(self.smoothedPath(:,1),self.smoothedPath(:,2),self.smoothedPath(:,3),'LineWidth',2,'Color','g','Parent',self.GetAxisHandle());
                title_h = get(self.GetAxisHandle(),'title');
                title_string=get(title_h,'String');
                title_string=[title_string,'. Initial(Red), Smoothed(Green)'];
                set(title_h,'String',title_string)   
            end
        end
%% Function CollisionCheck
% *Description:* Returns whether a collision occurs between an edge and a set
% of obstacles. Also gives the point of intersection. (P1=node,P2=parent node)
        function [collision,PInt] = CollisionCheck(self,P1,P2)
            self.collisionCheckCount = self.collisionCheckCount + 1;
            %default is that it is safe, then if a collision is found it is set to 1
            %and we return
            collision = false;
            if size(self.obs,1)==0
                PInt=inf;
                return;
            end
            % Calculate intercept point of a line and plane
            % ---------------------------------------------
            % Line equation
            r_var=[P1(1)-P2(1) P1(2)-P2(2) P1(3)-P2(3)];
            plane_equ = self.obstaclePlaneParameters;
            for i = 1:size(self.obs,3)
                % Plane * Line
                bottomof_t_var = plane_equ(i,1) * r_var(1) ...
                               + plane_equ(i,2) * r_var(2) ...
                               + plane_equ(i,3) * r_var(3);
                if bottomof_t_var == 0
                    % This should not happen because it means the point is exactly on the plane
                    collision = true;
                    return;
                end
                % Some variable
                t_var = ( plane_equ(i,1)*P1(1) + plane_equ(i,2)*P1(2) + plane_equ(i,3)*P1(3) + plane_equ(i,4)) ...
                        ./ bottomof_t_var;
                % Get intersection points
                PInt=[t_var.*-r_var(1)+P1(1),...
                      t_var.*-r_var(2)+P1(2),...
                      t_var.*-r_var(3)+P1(3)];
                % Check if intercept point lies within line segment
                sqrd_distbetweenpnts=(P2(1)-P1(1))^2+(P2(2)-P1(2))^2+(P2(3)-P1(3))^2;
                if (PInt(1)-P1(1))^2 + (PInt(2)-P1(2))^2 + (PInt(3)-P1(3))^2 < sqrd_distbetweenpnts && ...
                   (PInt(1)-P2(1))^2 + (PInt(2)-P2(2))^2 + (PInt(3)-P2(3))^2 < sqrd_distbetweenpnts;
                    % Test whether intercept point is within boundaries
                    % -------------------------------------------------
                    % Check if point lies within plane boundaries
                    % Ref: http://www.blackpawn.com/texts/pointinpoly/default.html
                    if PointInQuad(PInt,self.obs(:,:,i));
                        collision = true;
                        % No need to check anymore since there is a collision 
                        return;
                    end        
                end
            end
        end                          
        
%% Function DisplayTree
%  *Description:* This function displays the results of the search.
        function DisplayTree(self,cur_it)
            validtrees=0;
            colours=['b','k','c','m','y','g'];
            for t=1:size(self.rrt,2)
                if self.rrt(t).valid
                    validtrees = validtrees + 1;
                    % Plot new edges               
                    for i=2:size(self.rrt(t).parent,1)            
                        try delete(self.plotHandles(t).lines(i));end %#ok<TRYNC>
                        self.plotHandles(t).lines(i)=plot3([self.rrt(t).cords(self.rrt(t).parent(i),1),self.rrt(t).cords(i,1)] ...
                                                          ,[self.rrt(t).cords(self.rrt(t).parent(i),2),self.rrt(t).cords(i,2)] ...
                                                          ,[self.rrt(t).cords(self.rrt(t).parent(i),3),self.rrt(t).cords(i,3)],colours(mod(t,size(colours,2))+1)); %,'Color',[t/(treesMax*3) t/(treesMax*1) t/(treesMax*2)]); %[t/size(self.rrt,2),t/size(self.rrt,2),t/size(self.rrt,2)]);
                    end
                else
                    for i=2:size(self.rrt(t).parent,1)
                        try delete(self.plotHandles(t).lines(i));end %#ok<TRYNC>
                    end
                    try delete(self.plotHandles(t).points); end %#ok<TRYNC>
                end
            end
            % Plot title
            title(['Rapidly-Exploring Random Trees (Step: ', num2str(cur_it), '), No. of active trees = ',num2str(validtrees)]);
            drawnow;
        end
        
%% GetAxisHandle
%> @brief Check to ensure the axis exist before plotting to it, otherwise re setup the figure.
%> Returns the axis_h handle
%> @param self (handle) the class handle
%> @retval self (axes handle) handle to the main plotting axis
        function value = GetAxisHandle(self)            
            axisValid = false;            
            try  %#ok<TRYNC>
                % Ensure it is possible to get from axis_h. This will fail if the figure is closed
                get(self.axis_h,'Type');
                axisValid = true;
            end
            
            if isempty(self.axis_h) || ~axisValid                
                self.axis_h = get_axis_handle(self.figure_h);
                view(3);
                xlabel('x');ylabel('y');zlabel('z');
            end
            value = self.axis_h;
        end                
    end
    
    methods (Static)
%% First example wall obstacle with hole
        function wall = WallAtY(y,wallType)
            if wallType == 1
                wall(:,:,1)=[-0.50 y +0.50; 
                             +0.15 y +0.50; 
                             +0.15 y -0.50; 
                             -0.50 y -0.50];
                            
                wall(:,:,2)=[+0.35 y-0.05 +0.50; 
                             +0.50 y-0.05 +0.50; 
                             +0.50 y-0.05 -0.50; 
                             +0.35 y-0.05 -0.50];
                             
                wall(:,:,3)=[+0.15 y+0.05 +0.50; 
                             +0.35 y+0.05 +0.50; 
                             +0.35 y+0.05 +0.35; 
                             +0.15 y+0.05 +0.35];
                              
                wall(:,:,4)=[+0.15 y-0.15 +0.15; 
                             +0.35 y-0.15 +0.15; 
                             +0.35 y-0.15 -0.50; 
                             +0.15 y-0.15 -0.50];
              
            elseif wallType == 2
                wall(:,:,1)=[-0.50 y +0.50; 
                             -0.35 y +0.50; 
                             -0.35 y -0.50; 
                             -0.50 y -0.50];
                            
                wall(:,:,2)=[-0.15 y +0.50; 
                             +0.50 y +0.50; 
                             +0.50 y -0.50; 
                             -0.15 y -0.50];
                             
                wall(:,:,3)=[-0.35 y +0.50; 
                             -0.15 y +0.50; 
                             -0.15 y -0.15; 
                             -0.35 y -0.15];
                  
                wall(:,:,4)=[-0.35 y -0.35; 
                            -0.15 y -0.35; 
                            -0.15 y -0.50; 
                            -0.35 y -0.50];
                        
            end
        end
    end
end
%% Function Dist2edges
% *Description:* Given a starting point (P1) and an end point (P2) of a line, plus another
% point (P3), this function will find the closest point along the line
% (PInt) to P3 and also return the distance (Dist2PInt) between PInt and P3.
function [PIntE,d2edge] = Dist2edges(P1,P2,P3) % P1=Node, P2=Parent, P3=Random
    % Make sure there is some distance between P1 and P2 (the square of dist between)
    distbtwnAllNodes=(P2(:,1)-P1(:,1)).^2 + (P2(:,2)-P1(:,2)).^2 + (P2(:,3)-P1(:,3)).^2;
    % Check places with no distance between P1 and P2
    newindex=(distbtwnAllNodes==0);
    % Need the index so make this approximately zero
    distbtwnAllNodes(newindex)=eps;
    % Coordinates of closest point
    u = ((  P3(:,1)-P1(:,1)).*(P2(:,1)-P1(:,1)) ...
         + (P3(:,2)-P1(:,2)).*(P2(:,2)-P1(:,2)) ...
         + (P3(:,3)-P1(:,3)).*(P2(:,3)-P1(:,3))) ...
      ./distbtwnAllNodes;
    PIntE = [P1(:,1) + u.*(P2(:,1) - P1(:,1)) ...
           , P1(:,2) + u.*(P2(:,2) - P1(:,2)) ...
           , P1(:,3) + u.*(P2(:,3) - P1(:,3))];
    % If the point of intersection is not between the other two points in either
    % or the dist between them was zero, make the point to be at infinity
    PIntE((PIntE(:,1)-P1(:,1)).^2 + (PIntE(:,2)-P1(:,2)).^2 + (PIntE(:,3)-P1(:,3)).^2 > (P2(:,1)-P1(:,1)).^2 + (P2(:,2)-P1(:,2)).^2 + (P2(:,3)-P1(:,3)).^2|...
          (PIntE(:,1)-P2(:,1)).^2 + (PIntE(:,2)-P2(:,2)).^2 + (PIntE(:,3)-P2(:,3)).^2 > (P2(:,1)-P1(:,1)).^2 + (P2(:,2)-P1(:,2)).^2 + (P2(:,3)-P1(:,3)).^2|...
           newindex,:) = inf;
    % Calculate the distance to all nodes where PIntE is not infinite
    d2edge=zeros([size(PIntE,1),1]);
    d2edge(PIntE(:,1)==inf) = inf;
    d2edge(PIntE(:,1)~=inf) = sqrt((PIntE(PIntE(:,1)~=inf,1)-P3(:,1)).^2 ...
                                  +(PIntE(PIntE(:,1)~=inf,2)-P3(:,2)).^2 ...
                                  +(PIntE(PIntE(:,1)~=inf,3)-P3(:,3)).^2);        
end  
%% Function sameSide
%
% *Description:* sameSide Technique for working out if points a and b are
% on the same side of the line p1->p2 fast matlab implementation of code
% initially from  http://www.blackpawn.com/texts/pointinpoly/default.html
function Pin = SameSide(p1,p2,a,b)
    BminusA = [b-a]';
    P1minusA = [p1-a]';
    % Formula for cross product
    cp1 = [BminusA(2,:).*P1minusA(3,:)-BminusA(3,:).*P1minusA(2,:),...
           BminusA(3,:).*P1minusA(1,:)-BminusA(1,:).*P1minusA(3,:),...
           BminusA(1,:).*P1minusA(2,:)-BminusA(2,:).*P1minusA(1,:)];
    P2minusA = [p2-a]';
    cp2 = [BminusA(2,:).*P2minusA(3,:)-BminusA(3,:).*P2minusA(2,:),
           BminusA(3,:).*P2minusA(1,:)-BminusA(1,:).*P2minusA(3,:),
           BminusA(1,:).*P2minusA(2,:)-BminusA(2,:).*P2minusA(1,:)];
    % This is quicker than dot product  
    if sum(cp1'.*cp2)>=0
        Pin=1;
    else
        Pin=0;
    end
end
%% Function NearestNeighbour
% *Description:* This function finds the nearest node or edge.
function [d2nodes,d2edges] = NearestNeighbour(new_pnt,rrtLocal)
    %go through each tree and find vector of dist to edge and nodes
    for t=1:size(rrtLocal,2)
        if rrtLocal(t).valid
            % Distance to all nodes 
            d2nodes(t).vals=sqrt((rrtLocal(t).cords(:,1)-new_pnt(1)).^2+...
                                 (rrtLocal(t).cords(:,2)-new_pnt(2)).^2+...
                                 (rrtLocal(t).cords(:,3)-new_pnt(3)).^2); %#ok<AGROW>
            % Distance to all edges (if there are any)
            if size(rrtLocal(t).cords,1)>1
                lineStarts = rrtLocal(t).cords(2:end,:);
                lineEnds = rrtLocal(t).cords(rrtLocal(t).parent(2:end),:);            
                [d2edges(t).node,d2edges(t).vals] = Dist2edges(lineStarts,lineEnds,new_pnt); %#ok<AGROW>
            else
                d2edges(t).node = inf(1,3); %#ok<AGROW>
                d2edges(t).vals = inf; %#ok<AGROW>
            end
        end
    end
end
%% Function PointInQuad
% *Description:* Is point of intersection inside the obstacle uses sameSide
% usually used for checking a point in a mesh (triangle) but here we do the
% same for a 4 point plannar object   
function test_plane = PointInQuad(Pint,singlePlane)
    %% this is slightly faster
    test_plane=0;
    if SameSide(Pint,singlePlane(3,:,:),singlePlane(1,:,:),singlePlane(2,:,:)) 
        if SameSide(Pint,singlePlane(4,:,:),singlePlane(2,:,:),singlePlane(3,:,:)) 
            if SameSide(Pint,singlePlane(1,:,:),singlePlane(3,:,:),singlePlane(4,:,:)) 
                if SameSide(Pint,singlePlane(2,:,:),singlePlane(4,:,:),singlePlane(1,:,:))
                    test_plane=1;
                end
            end
        end
    end
end
