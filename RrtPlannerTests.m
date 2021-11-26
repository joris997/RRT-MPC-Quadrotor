%> Tests for the RrtPlanner class 
classdef RrtPlannerTests < TestCase % You must have mlUnit_2008a for this to work properly
    methods        
%% testRrtPlannerNoParam
        function testRrtPlannerNoParam(self)            
            rrt = RrtPlanner();
            self.assertEquals(isa(rrt,'RrtPlanner'),1);
            delete(rrt);
        end
        
%% testRrtPlannerTreesMax
        function testRrtPlannerTreesMax(self)            
            treesMax = 27;
            rrt = RrtPlanner(treesMax);            
            self.assertEquals(rrt.treesMax == treesMax,1);
            delete(rrt);
        end
        
        
%% testRrtPlannerTreesMaxSeedsPerAxis
        function testRrtPlannerTreesMaxSeedsPerAxis(self)            
            treesMax = 27;
            seedsPerAxis = 4;
            rrt = RrtPlanner(treesMax,seedsPerAxis);
            self.assertEquals(rrt.treesMax == treesMax,1);
            self.assertEquals(rrt.seedsPerAxis == floor(power(treesMax-2,1/3)),1);
            floor(power(treesMax-2,1/3));
            delete(rrt);
        end
%% testRrtPlannerTreesMaxSeedsPerAxisWalls
        function testRrtPlannerTreesMaxSeedsPerAxisWalls(self)         
            treesMax = 28;
            seedsPerAxis = 5;
            wallCount = 6;
            rrt = RrtPlanner(treesMax,seedsPerAxis,wallCount);
            self.assertEquals(rrt.treesMax == treesMax,1);
            self.assertEquals(rrt.seedsPerAxis == floor(power(treesMax-2,1/3)),1);
            self.assertEquals(rrt.wallCount == wallCount,1);
            self.assertEquals(size(rrt.obs,1) == 4 && size(rrt.obs,2) == 3 && 0 < size(rrt.obs,3) ,1);                        
            self.assertEquals(size(rrt.rrt,2) == treesMax,1);            
            delete(rrt);
        end
        
%% testRrtPlannerTreesMaxSeedsPerAxisObstacleFile
        function testRrtPlannerTreesMaxSeedsPerAxisObstacleFile(self)         
            treesMax = 28;
            seedsPerAxis = 5;
            obstacleFilename = 'obstacles.txt';
            rrt = RrtPlanner(treesMax,seedsPerAxis,obstacleFilename);
            self.assertEquals(rrt.treesMax == treesMax,1);
            self.assertEquals(rrt.seedsPerAxis == floor(power(treesMax-2,1/3)),1);
            self.assertEquals(size(rrt.obs,1) == 4 && size(rrt.obs,2) == 3 && 0 < size(rrt.obs,3) ,1);
            self.assertEquals(strcmp(rrt.obstacleFilename,obstacleFilename),1);
            delete(rrt);
        end   
%% testSetStartAndGoal
        function testSetStartAndGoal(self)            
            rrt = RrtPlanner();
            rrt.SetStart([0.1,0.1,0.1]);
            rrt.SetGoal([0.2,0.2,0.2]);
            self.assertEquals(all(rrt.start == 0.1),1);
            self.assertEquals(all(rrt.goal == 0.2),1);
            delete(rrt);
        end            
%% testSetAvoidObstacles
        function testSetAvoidObstacles(self)            
            rrt = RrtPlanner();
            rrt.SetAvoidObstacles();            
            self.assertEquals(rrt.avoidObstacles,1);
            rrt.SetAvoidObstacles(false);
            self.assertEquals(~rrt.avoidObstacles,1);
            rrt.SetAvoidObstacles(true);
            self.assertEquals(rrt.avoidObstacles,1);
            delete(rrt);
        end            
        
%% testRun
        function testRun(self)            
            treesMax = 28;
            seedsPerAxis = 5;
            wallCount = 2;
            rrt = RrtPlanner(treesMax,seedsPerAxis,wallCount);
            rrt.Run();
            self.assertEquals(~isempty(rrt.path),1);
            delete(rrt);
        end
        
%% testRunLoadedObstaclesEasy
        function testRunLoadedObstaclesEasy(self)            
            treesMax = 29;
            seedsPerAxis = 3;            
            obstacleFilename = 'obstacles.txt';
            rrt = RrtPlanner(treesMax,seedsPerAxis,obstacleFilename);
            rrt.Run();
            self.assertEquals(~isempty(rrt.path),1);
            delete(rrt);
        end
%% testRunLoadedObstaclesMedium
        function testRunLoadedObstaclesMedium(self)            
            treesMax = 29;
            seedsPerAxis = 3;
            obstacleFilename = 'obstacles2.txt';
            rrt = RrtPlanner(treesMax,seedsPerAxis,obstacleFilename);
            rrt.Run();
            self.assertEquals(~isempty(rrt.path),1);
            delete(rrt);
        end
        
%% testRunLoadedObstaclesHard
        function testRunLoadedObstaclesHard(self)                        
            seedsPerAxis = 7;
            treesMax = seedsPerAxis^3*3+2;
            obstacleFilename = 'obstacles3.txt';
            rrt = RrtPlanner(treesMax,seedsPerAxis,obstacleFilename);
            rrt.drawingSkipsPerDrawing = 50;
            rrt.Run();
            self.assertEquals(~isempty(rrt.path),1);
            delete(rrt);
        end
        
%% testRunLoadedObstaclesTunnel
        function testRunLoadedObstaclesTunnel(self)                        
            seedsPerAxis = 7;
            treesMax = seedsPerAxis^3*3+2;
            obstacleFilename = 'obstacles4.txt';
            rrt = RrtPlanner(treesMax,seedsPerAxis,obstacleFilename);
            rrt.drawingSkipsPerDrawing = 10;
            rrt.Run();
            self.assertEquals(~isempty(rrt.path),1);
            delete(rrt);
        end
        
%% testChangeObstacleFilename
        function testChangeObstacleFilename(self)                        
            seedsPerAxis = 7;
            treesMax = seedsPerAxis^3*3+2;
            obstacleFilename = 'obstacles3.txt';
            rrt = RrtPlanner(treesMax,seedsPerAxis,obstacleFilename);
            % Change obstacles
            rrt.SetObstacleFilename('obstacles.txt');
            
            rrt.drawingSkipsPerDrawing = 50;
            rrt.Run();
            
            self.assertEquals(~isempty(rrt.path),1);
            self.assertEquals(~isempty(rrt.smoothedPath),1);
                        
            fig_h = plot3(rrt.path(:,1),rrt.path(:,2),rrt.path(:,3),'r*');
            plot3(rrt.smoothedPath(:,1),rrt.smoothedPath(:,2),rrt.smoothedPath(:,3),'g*');
            
            
            delete(fig_h);
            delete(rrt);                       
        end        
    end                   
end