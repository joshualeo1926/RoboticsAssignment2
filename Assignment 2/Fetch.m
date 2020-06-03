classdef Fetch < handle
    properties
        model;
        base = transl(0,0,0);
        workspace = [-2 2 -2 2 -2 2];
        name = 'Robot';
        scale = 0;
        sequence;
        baseItter = 1;
        armItter = 1;
        collisionRadius = 0.29;
        attachedObjects = [];
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
            steps = 50;
            basePos = zeros(4, 4, steps);
            s = lspb(0,1,steps);
            for i = 1:steps
                targetPos = (1-s(i))*initialPos + s(i)*finalPos;
                basePos(:, :, i) = transl(targetPos(1, 4), targetPos(2, 4), targetPos(3, 4));
                basePos(1:2, 1:2, i) = initialPos(1:2, 1:2);
            end 
        end
        
        function qMatrix = Move(self, targetPos)
            steps = 50;
            initialPos = self.model.getpos;
            finalPos = self.model.ikcon(targetPos, initialPos);
            s = lspb(0,1,steps);
            qMatrix = nan(steps, 7);
            for i = 1:steps
                qMatrix(i, :) = (1-s(i))*initialPos + s(i)*finalPos;
            end
        end
        
        function qMatrix = MoveJointState(self, jointState)
            steps = 50;
            initialPos = self.model.getpos;
            finalPos = jointState;
            s = lspb(0,1,steps);
            qMatrix = nan(steps, 7);
            for i=1:steps
                qMatrix(i, :) = (1-s(i))*initialPos + s(i)*finalPos;
            end
        end
        
        % NOT USED
        function [isCollision, intersectP, i] = IsArmCollision(self, pose, environment)
            rCount = 0;
            stopMotion = 0;
            for j = 1:numel(environment)
                [result, intersectP, i] = IsCollision(self.model, pose, environment(j).f, ...
                    environment(j).verts, environment(j).fn);
                if(result == 0)
                    rCount = rCount + 1;
                elseif(result == 1)
                    disp(['colliding with ', environment(j).name{1}])
                    isCollision = 1;
                    break
                end
                environmentSize = size(environment);
                if rCount == environmentSize(2) && stopMotion == 0
                    isCollision = 0;
                end
            end  
        end
        
        function qMatrix = ArmRMRCPos(self, targetPos)
            initialPos = self.model.fkine(self.model.getpos);
            t = 0.5;   
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
                
                invJ = inv(J'*J + lambda *eye(7))*J';                        
                qdot(i,:) = (invJ*xdot)';                                             
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

        function qMatrix = ArmRMRCJoints(self, targetJoints)
            initialPos = self.model.fkine(self.model.getpos);
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
                
                invJ = inv(J'*J + lambda *eye(7))*J';                        
                qdot(i,:) = (invJ*xdot)';                                             
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
            for i = 1:numel(environment)
                for ang = 0:0.3:2*pi
                    xp=self.collisionRadius*cos(ang);
                    yp=self.collisionRadius*sin(ang);
                    zMin = self.model.base(3, 4) - 0.5;
                    zMax = self.model.base(3, 4) + 0.43;
                    for faceIndex = 1:size(environment(i).f, 1) 
                        vertOnPlane = environment(i).mesh.Vertices(environment(i).f(faceIndex,1)',:);
                        [intersectP,check] = LinePlaneIntersection(environment(i).fn(faceIndex,:), vertOnPlane, [xp yp zMin], [xp yp zMax]); 
                        if check == 1 && IsIntersectionPointInsideTriangle(intersectP, environment(i).mesh.Vertices(environment(i).f(faceIndex,:)',:))
                            result = true;
                            return
                        end
                    end
                end
            end
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