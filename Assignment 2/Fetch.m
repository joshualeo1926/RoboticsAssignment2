classdef Fetch < handle
    properties
        model;
        base = transl(0,0,0);
        workspace = [-2 2 -2 2 -2 2];
        name = 'Robot';
        scale = 0.2;
    end
    
    methods
        function self = Fetch(base, workspace, name)
            if 0 < nargin
                self.base = base;
                self.workspace = workspace;
                self.name = name;
            end
            
            self.GetFetchRobot();
        end
        
        function GetFetchRobot(self)
            pause(0.001);
        	link1 = Link('d',0.107511,'a',0.120238,'alpha',pi/2,'qlim',[deg2rad(-92),deg2rad(92)]); %shoulderpan
            link2 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[deg2rad(-87),deg2rad(70)], 'offset', deg2rad(90)); % shoulder lift
            link3 = Link('d',0.179933,'a',0,'alpha',-pi/2); % upperarm roll
            link4 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[deg2rad(-129),deg2rad(129)]); %elbow flex
            link5 = Link('d',0.15712,'a',0,'alpha',pi/2); %forearm roll
            link6 = Link('d',0.051119,'a',0,'alpha',-pi/2,'qlim',[deg2rad(-125),deg2rad(125)]); %wrist flex
            link7 = Link('d',0.04237,'a',0,'alpha',0); %wrist roll
            qMatrix = [0 0 0 0 0 0 0];
            
            self.model = SerialLink([link1 link2 link3 link4 link5 link6 link7], 'name', self.name, 'base', self.base);
            self.model.plot(qMatrix, 'workspace', self.workspace, 'scale', self.scale, 'noarrow');
            %self.model.plot(qMatrix, 'workspace', self.workspace,'noarrow');
        end
        
        function Move(self, pos)
            steps = 50;
            initialPos = self.model.getpos;
            finalPos = self.model.ikcon(pos, initialPos);
            s = lspb(0,1,steps);
            qMatrix = zeros(steps, 7);
            for i=1:steps
                qMatrix(i, :) = (1-s(i))*initialPos(1, 4) + s(i)*finalPos;
            end
            self.model.plot(qMatrix, 'noarrow')
        end
    end
end