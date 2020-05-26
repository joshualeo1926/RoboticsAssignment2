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
        
        function [actionComplete, basePos] = MoveBase(self, pos)
            initialPos = self.base;
            steps = 50;
            s = lspb(0,1,steps);
            if self.baseItter <= steps
                targetPos = (1-s(self.baseItter))*initialPos + s(self.baseItter)*pos;
                basePos = transl(targetPos(1, 4), targetPos(2, 4), targetPos(3, 4));
                actionComplete = 0;
                self.baseItter = self.baseItter + 1;
            else
                targetPos = (1-s(steps))*initialPos + s(steps)*pos;
                basePos = transl(targetPos(1, 4), targetPos(2, 4), targetPos(3, 4));
                self.baseItter = 1;
                actionComplete = 1;
            end 
        end
        
        function [actionComplete, qMatrix] = Move(self, pos)
            steps = 50;
            initialPos = self.model.getpos;
            finalPos = self.model.ikcon(pos, initialPos);
            s = lspb(0,1,steps);
            if self.armItter <= steps
                qMatrix = (1-s(self.armItter))*initialPos + s(self.armItter)*finalPos;
                actionComplete = 0;
                self.armItter = self.armItter + 1;
            else
                qMatrix = (1-s(steps))*initialPos + s(steps)*finalPos;
                actionComplete = 1;
                self.armItter = 1;        
            end 

        end
        
        function [isCollision, intersectP] = IsArmCollision(self, pose, environment)
            rCount = 0;
            stopMotion = 0;
            for j = 1:numel(environment)
                [result, intersectP] = IsCollision(self.model, pose, environment(j).f, ...
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
        
        function Move2(self, pos)
            initialPos = self.model.fkine(self.model.getpos);
            t = 10;   
            deltaT = 0.01;   
            steps = t/deltaT;  
            delta = 2*pi/steps; 
            epsilon = 0.1;    
            W = diag([1 1 1 0.1 0.1 0.1]);    

            m = zeros(steps,1);  
            qMatrix = zeros(steps,7);    
            qdot = zeros(steps,7);
            theta = zeros(3,steps); 
            x = zeros(3,steps);
            positionError = zeros(3,steps);
            angleError = zeros(3,steps);

            s = lspb(0,1,steps);   
            for i=1:steps
                x(1,i) = (1-s(i))*initialPos(1, 4) + s(i)*pos(1, 4);
                x(2,i) = (1-s(i))*initialPos(2, 4) + s(i)*pos(2, 4);
                x(3,i) = 0.5 + 0.2*sin(i*delta);
                theta(1,i) = 0;
                theta(2,i) = pi/2;
                theta(3,i) = 0;
            end
            
            plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)
            
            T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];
            q0 = zeros(1,7);               
            qMatrix(1,:) = self.model.ikcon(T,q0);                        

            % 1.4) Track the trajectory with RMRC
            for i = 1:steps-1
                T = self.model.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
                deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
                S = Rdot*Ra';                                                           % Skew symmetric!
                linear_velocity = (1/deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
                xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
                J = self.model.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
                m(i) = sqrt(det(J*J'));
                if m(i) < epsilon  % If manipulability is less than given threshold
                    lambda = (1 - m(i)/epsilon)*5E-2;
                else
                    lambda = 0;
                end
                
                invJ = inv(J'*J + lambda *eye(7))*J';                                   % DLS Inverse
                qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
                for j = 1:7                                                             % Loop through joints 1 to 6
                    if qMatrix(i,j) + deltaT*qdot(i,j) < self.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; % Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > self.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; % Stop the motor
                    end
                end
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
                positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
                angleError(:,i) = deltaTheta;                                           % For plotting
            end

            % 1.5) Plot the results
            figure(1)
            %plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)
            self.model.plot(qMatrix,'trail','r-')

        end
    end
end