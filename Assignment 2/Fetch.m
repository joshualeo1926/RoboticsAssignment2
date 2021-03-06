classdef Fetch < handle
    properties
        model;
        base = transl(0,0,0);
        workspace = [-2 2 -2 2 -2 2];
        name = 'Robot';
        scale = 0;
        collisionRadius = 0.29;
        attachedObjects = [];
        collision = true;
    end

    methods
        function self = Fetch(base, workspace, name)
            if 0 < nargin
                self.base = base;
                self.workspace = workspace;
                self.name = name;
            end
            self.GetFetchRobot();
            self.PlotAndColourRobot();
        end
        
        function GetFetchRobot(self)
            pause(0.001);
        	link1 = Link('d',0.107511,'a',0.120238,'alpha',pi/2,'qlim',[deg2rad(-92),deg2rad(92)]); %shoulderpan
            link2 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[deg2rad(-87),deg2rad(70)], 'offset', deg2rad(90)); % shoulder lift
            link3 = Link('d',0.35,'a',0,'alpha',-pi/2); % upperarm roll
            link4 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[deg2rad(-129),deg2rad(129)]); %elbow flex
            link5 = Link('d',0.315,'a',0,'alpha',pi/2); %forearm roll
            link6 = Link('d',0,'a',0,'alpha',-pi/2,'qlim',[deg2rad(-125),deg2rad(125)]); %wrist flex
            link7 = Link('d',0.15,'a',0,'alpha',0); %wrist roll
            
            self.model = SerialLink([link1 link2 link3 link4 link5 link6 link7], 'name', self.name, 'base', self.base);
        end
        
        function PlotAndColourRobot(self)
            for linkIndex = 0:self.model.n
                currentFile = mfilename( 'fullpath' );
                [pathstr,~,~] = fileparts( currentFile );
                plyPath = fullfile(pathstr , '..', 'PLY', ['L',  num2str(linkIndex), '.ply']);
                [f, v, d] = plyread(plyPath,'tri');
                self.model.faces{linkIndex + 1} = f;
                self.model.points{linkIndex + 1} = v;
                Data{linkIndex + 1} = d;
            end 
            % Display robot
            figure(1)
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [Data{linkIndex+1}.vertex.red ...
                                                                  , Data{linkIndex+1}.vertex.green ...
                                                                  , Data{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
        
        function basePos = MoveBase(self, finalPos)
            initialPos = self.model.base;
            steps = 25;
            basePos = zeros(4, 4, steps);
            s = lspb(0,1,steps);
            for i = 1:steps
                targetPos = (1-s(i))*initialPos + s(i)*finalPos;
                basePos(:, :, i) = transl(targetPos(1, 4), targetPos(2, 4), targetPos(3, 4));
                basePos(1:2, 1:2, i) = initialPos(1:2, 1:2);
            end 
        end
        
        function qMatrix = Move(self, targetPos, varargin)
            oldQlim = self.model.qlim;
            if nargin>2
                self.model.qlim = varargin{1};
            end
            steps = 25;
            initialPos = self.model.getpos;
            finalPos = self.model.ikcon(targetPos, initialPos);
            s = lspb(0,1,steps);
            qMatrix = nan(steps, 7);
            for i = 1:steps
                qMatrix(i, :) = (1-s(i))*initialPos + s(i)*finalPos;
            end
            self.model.qlim = oldQlim;
        end
        
        function qMatrix = MoveJointState(self, jointState)
            steps = 25;
            initialPos = self.model.getpos;
            finalPos = jointState;
            s = lspb(0,1,steps);
            qMatrix = nan(steps, 7);
            for i=1:steps
                qMatrix(i, :) = (1-s(i))*initialPos + s(i)*finalPos;
            end
        end
        
        function [isCollision, intersectP, i, j] = IsArmCollision(self, qMatrix, environment, checkType)
            if self.collision
                for k = 1:size(qMatrix, 1)              
                    rCount = 0;
                    stopMotion = 0;
                    for j = 1:numel(environment)
                        [result, intersectP, i] = IsCollision(self.model, qMatrix(k, :), environment(j).f, ...
                            environment(j).mesh.Vertices, environment(j).fn, true, checkType);
                        if(result == 0)
                            rCount = rCount + 1;
                        elseif(result == 1)
                            isCollision = 1;
                            if i == 7 && strcmp(environment(j).name{1}, 'WorkBench2') || strcmp(environment(j).name{1}, 'WorkBench')
                                isCollision = 0;
                            end
                            break
                        end
                        environmentSize = size(environment);
                        if rCount == environmentSize(2) && stopMotion == 0
                            isCollision = 0;
                        end
                    end  
                end
            else
                isCollision = 0;
                intersectP = [0 0 0];
                i = 1;
                j = 1;
            end
        end
        
        function qMatrix = ArmRMRCPos(self, targetPos, varargin)
            oldQlim = self.model.qlim;
            if nargin>2
                self.model.qlim = varargin{1};
            end
            if nargin>3
                initialPos = varargin{2};
            else
                initialPos = self.model.fkine(self.model.getpos);
            end
            t = 1;   %0.5
            deltaT = 0.05;   
            steps = t/deltaT;  
            epsilon = 0.1;    
            W = diag([1 1 1 1 1 1]);    
            initRPY = tr2rpy(initialPos);
            targetRPY = tr2rpy(targetPos);
            m = zeros(steps,1);  
            qMatrix = zeros(steps,7);    
            qdot = zeros(steps,7);
            theta = zeros(3,steps); 
            x = zeros(3,steps);

            s = lspb(0,1,steps);   
            for i=1:steps
                x(1,i) = (1-s(i))*initialPos(1, 4) + s(i)*targetPos(1, 4);
                x(2,i) = (1-s(i))*initialPos(2, 4) + s(i)*targetPos(2, 4);
                x(3,i) = (1-s(i))*initialPos(3, 4) + s(i)*targetPos(3, 4);
                theta(1,i) = (1-s(i))*initRPY(1) + s(i)*targetRPY(1);
                theta(2,i) = (1-s(i))*initRPY(2) + s(i)*targetRPY(2);
                theta(3,i) = (1-s(i))*initRPY(3) + s(i)*targetRPY(3);
            end

            qMatrix(1,:) = self.model.getpos;

            for i = 1:steps-1
                T = self.model.fkine(qMatrix(i,:)); 
                deltaX = x(:,i+1) - T(1:3,4);
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1)); 
                Ra = T(1:3,1:3); 
                Rdot = (1/deltaT)*(Rd - Ra); 
                S = Rdot*Ra';
                linear_velocity = (1/deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)]; 
        
                xdot = W*[linear_velocity;angular_velocity];                          	
                J = self.model.jacob0(qMatrix(i,:));
                m(i) = sqrt(det(J*J'));
                if m(i) < epsilon 
                    lambda = (1 - m(i)/epsilon)*5E-2;
                else
                    lambda = 0;
                end
                
                N = null(J);
                invJ = pinv(J'*J + lambda *eye(7))*J';                     
                qdot(i,:) = (invJ*xdot+N)';                                             
                for j = 1:7
                    if j == 3 || j == 5 || j == 7
                        if qMatrix(i,j) >= 180
                           qMatrix(i,j) = mod(qMatrix(i,j), -180);
                        elseif qMatrix(i,j) <= -180
                           qMatrix(i,j) = mod(qMatrix(i,j), -180);
                        end
                    end
                    if qMatrix(i,j) + deltaT*qdot(i,j) < self.model.qlim(j,1)   
                        qdot(i,j) = 0;
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > self.model.qlim(j,2)
                        qdot(i,j) = 0;
                    end
                end
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);  
            end
            self.model.qlim = oldQlim;
        end

        function qMatrix = ArmRMRCJoints(self, targetJoints, varargin)
            if nargin>2
                initialPos = self.model.fkine(varargin{1});
            else
                initialPos = self.model.fkine(self.model.getpos);
            end
            targetPos = self.model.fkine(targetJoints);
            t = 0.5;   
            deltaT = 0.01;   
            steps = t/deltaT;  
            epsilon = 0.1;    
            W = diag([1 1 1 1 1 1]);    
            initRPY = tr2rpy(initialPos);
            targetRPY = tr2rpy(targetPos);
            m = zeros(steps,1);  
            qMatrix = zeros(steps,7);    
            qdot = zeros(steps,7);
            theta = zeros(3,steps); 
            x = zeros(3,steps);
            s = lspb(0,1,steps); 
            for i=1:steps
                x(1,i) = (1-s(i))*initialPos(1, 4) + s(i)*targetPos(1, 4);
                x(2,i) = (1-s(i))*initialPos(2, 4) + s(i)*targetPos(2, 4);
                x(3,i) = (1-s(i))*initialPos(3, 4) + s(i)*targetPos(3, 4);
                theta(1,i) = (1-s(i))*initRPY(1) + s(i)*targetRPY(1);
                theta(2,i) = (1-s(i))*initRPY(2) + s(i)*targetRPY(2);
                theta(3,i) = (1-s(i))*initRPY(3) + s(i)*targetRPY(3);
            end

            qMatrix(1,:) = self.model.getpos;

            for i = 1:steps-1
                T = self.model.fkine(qMatrix(i,:)); 
                deltaX = x(:,i+1) - T(1:3,4);
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1)); 
                Ra = T(1:3,1:3); 
                Rdot = (1/deltaT)*(Rd - Ra); 
                S = Rdot*Ra';
                linear_velocity = (1/deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)]; 
        
                xdot = W*[linear_velocity;angular_velocity];                          	
                J = self.model.jacob0(qMatrix(i,:));
                m(i) = sqrt(det(J*J'));
                if m(i) < epsilon 
                    lambda = (1 - m(i)/epsilon)*5E-2;
                else
                    lambda = 0;
                end
                
                N = null(J);
                invJ = pinv(J'*J + lambda *eye(7))*J';                     
                qdot(i,:) = (invJ*xdot+N)';                                                 
                for j = 1:7                                        
                    if qMatrix(i,j) + deltaT*qdot(i,j) < self.model.qlim(j,1)   
                        qdot(i,j) = 0;
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > self.model.qlim(j,2)
                        qdot(i,j) = 0;
                    end
                end
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);  
            end
        end
        
        function result = CheckBaseCollision(self, environment) 
            if self.collision
                result = false;
                for i = 1:numel(environment)
                    for j = 1:numel(environment(i).mesh.Vertices(:, 1))
                        if sqrt((environment(i).mesh.Vertices(j, 1) - self.model.base(1, 4))^2 + (environment(i).mesh.Vertices(j, 2) - self.model.base(2, 4))^2) <= self.collisionRadius ...
                                && environment(i).mesh.Vertices(j, 3) >= self.model.base(3, 4) - 0.5 && environment(i).mesh.Vertices(j, 3) <= self.model.base(3, 4) + 0.43
                            result = true;
                            return
                        end
                    end
                end
                [isCollision, intersectP, i, j] = self.IsArmCollision(self.model.getpos, environment, 0);
                if isCollision
                    result = true;
                    return
                end
            else
                result = false;
            end
        end
        
        function qMatrix = retreat(self)
            qMatrix1 = self.ArmRMRCPos(self.model.fkine(self.model.getpos) * transl(0, -0.1, 0));
            currentPos = qMatrix1(size(qMatrix1, 1), :);
            qMatrix2 = self.ArmRMRCJoints(deg2rad([-60 65 0 30 161 30 rad2deg(currentPos(7))]), currentPos);
            currentPos = qMatrix2(size(qMatrix2, 1), :);
            qMatrix3 = self.ArmRMRCJoints(deg2rad([92 65 0 30 161 30 rad2deg(currentPos(7))]), currentPos);
            currentPos = qMatrix3(size(qMatrix3, 1), :);
            qMatrix4 = self.ArmRMRCJoints(deg2rad([92 30 0 -100 0 80 0]), currentPos);
            currentPos = qMatrix4(size(qMatrix4, 1), :);
            qMatrix5 = self.ArmRMRCJoints(deg2rad([92 -50 0 -115 0 15 0]), currentPos);
            currentPos = qMatrix5(size(qMatrix5, 1), :);
            qMatrix6 = self.ArmRMRCJoints(deg2rad([92 -80 0 -100 0 85 0]), currentPos);
            qMatrix = [qMatrix1; qMatrix2; qMatrix3; qMatrix4; qMatrix5; qMatrix6];
        end
        
        function AttachObject(self, object)
            self.attachedObjects = [object];
        end
        
        function DetachObject(self)
            self.attachedObjects = [];
        end
        
        function UpdateObjectPos(self)
            for i=1:numel(self.attachedObjects)
                endEffectorPos = self.model.fkine(self.model.getpos);
                updatedPoints = [endEffectorPos * [self.attachedObjects(i).oriVerts,ones(self.attachedObjects(i).vertexCount,1)]']';  
                self.attachedObjects(i).mesh.Vertices = updatedPoints(:,1:3); 
            end
        end
    end
end
        
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                      % intersectP is in Triangle
end