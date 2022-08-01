classdef VrepConnector
    % This class is finished with the pose of table and append force
    properties
        sim;				%Similar to fd
        clientID;			%Used for server connection and server requests
        robot_joints = []	%List of joint handles
        sensorhandle=[]
        handle_table=[];
        step_time_vrep;		%Integration step used for simulation
    end
    
    methods
        function obj = VrepConnector(port, step_time_vrep)
            addpath vrep_lib/;						%Adding the APIs to the path
            obj.step_time_vrep = step_time_vrep;	
            obj.sim = remApi('remoteApi');			%RemoteAPI object
            obj.sim.simxFinish(-1);
            obj.clientID = obj.sim.simxStart('127.0.0.1', port, true, true, 5000, 5);
            if (obj.clientID > -1)
                disp('Connected to simulator');
            else
                disp('Error in connection');
            end
            dt = 0.01;
            % simulation duration
            duration = 10;
            % enable the synchronous mode on the client: (integration step on call)
             obj.sim.simxSetFloatingParameter(obj.clientID,obj.sim.sim_floatparam_simulation_time_step,dt,obj.sim.simx_opmode_oneshot_wait);

            obj.sim.simxSynchronous(obj.clientID, true);
            % start the simulation:
            obj.sim.simxStartSimulation(obj.clientID, obj.sim.simx_opmode_blocking);
            
            
%             for i = 1:5
%                 [~,obj.robot_joints(i)] = obj.sim.simxGetObjectHandle(obj.clientID, strcat('Cuboid',int2str(i)), obj.sim.simx_opmode_blocking);
%                 FUKCYOUTT=obj.robot_joints(i)
%             end
            
            for i = 1:7 
                [~,obj.robot_joints(i)] = obj.sim.simxGetObjectHandle(obj.clientID, strcat('LBR_iiwa_14_R820_joint',int2str(i)), obj.sim.simx_opmode_blocking);
            end
            
            [~,obj.handle_table(1)] = obj.sim.simxGetObjectHandle(obj.clientID, strcat('down'), obj.sim.simx_opmode_blocking);
%             [~,obj.handle_table(1)] = obj.sim.simxGetObjectHandle(obj.clientID, strcat('hand'), obj.sim.simx_opmode_blocking);
            
%             [~,obj.handle_table] = obj.sim.simxGetObjectHandle(obj.clientID, strcat('down'), obj.sim.simx_opmode_blocking);
            for i = 1:7
                [~, joint_pos] = obj.sim.simxGetJointPosition(obj.clientID, obj.robot_joints(i), obj.sim.simx_opmode_streaming);
            end
            
            
           [~,obj.sensorhandle] = obj.sim.simxGetObjectHandle(obj.clientID,['LBR_iiwa_14_R820_connection'],obj.sim.simx_opmode_blocking);
           [~, vel_obj,~]=obj.sim.simxGetObjectVelocity(obj.clientID,obj.handle_table(1),obj.sim.simx_opmode_streaming );

        end
%%        
        function R_F=Getforce(obj)  % this does not work, the result is always 0 0 0 
%             simxReadForceSensor(id,external_force_sensor,vrep.simx_opmode_buffer);
            [~,force_sensor_state,R_F,external_torque_vec]=obj.sim.simxReadForceSensor(obj.clientID,obj.sensorhandle,obj.sim.simx_opmode_buffer);
        end
        
        function hands=Gethandles(obj)
            hands = obj.robot_joints;
        end 
        
        
        function Close(obj)
            obj.sim.simxStopSimulation(obj.clientID, obj.sim.simx_opmode_blocking);
            obj.sim.simxFinish(-1);
            obj.sim.delete();
        end
        
        function ApplyControl(obj, u, delta_t)
            for i = 1:7
                obj.sim.simxSetJointTargetVelocity(obj.clientID, obj.robot_joints(i), u(i), obj.sim.simx_opmode_oneshot);
            end
            for i = 1:(delta_t/obj.step_time_vrep)				%Number of integrations in delta_t
                obj.sim.simxSynchronousTrigger(obj.clientID);	%Triggering the integration
                % To overcome delay in values according to (Remote API modus operandi) document  
            end
            obj.sim.simxGetPingTime(obj.clientID);				%Synchronizing
        end
%%
        function ApplyPosi(obj, u)
            for i = 1:7
                gg=1;
                obj.sim.simxSetJointTargetPosition(obj.clientID, obj.robot_joints(i), u(i), obj.sim.simx_opmode_oneshot);
            end
        end
         
       function ApplyPosi2(obj, u)
            for i = 1:7
                gg=1;
                obj.sim.simxSetJointPosition(obj.clientID, obj.robot_joints(i), u(i), obj.sim.simx_opmode_oneshot);
            end
         end
        
        
        function Applyhand(obj, x,y,z)
 
             %obj.sim.simxSetJointPosition(obj.clientID, obj.hand, u, obj.sim.simx_opmode_oneshot);
        obj.sim.simxCallScriptFunction(obj.clientID, 'Dummy', obj.sim.sim_scripttype_childscript, 'addForceAndTorque_function', [x,y,z],[0,0,0],[],[], obj.sim.simx_opmode_blocking);
         %[res retInts retFloats retStrings retBuffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'createDummy_function',[],[0.1 0.2 0.3],'MyDummyName',[],vrep.simx_opmode_blocking);
      
        end        
         
        function Applyhand2(obj, x,y,z)
            obj.sim.simxCallScriptFunction(obj.clientID, 'Dummy', obj.sim.sim_scripttype_childscript, 'applyForce', [x,y,z],[0,0,0],[],[], obj.sim.simx_opmode_blocking);
        end        
        
        
        function q = GetState(obj)
            q = zeros(7,1);
            for i=1:7
                [~, q(i)] = obj.sim.simxGetJointPosition(obj.clientID, obj.robot_joints(i), obj.sim.simx_opmode_buffer);
            end
        end
        
        function q_dot = GetV(obj)
            q_dot = zeros(7,1);
            for i=1:7
                [~, q_dot(i)] = obj.sim.simxGetObjectFloatParameter(obj.clientID,obj.robot_joints(i),2012,obj.sim.simx_opmode_buffer);
            end
        end      
        
        function time = GetLastCmdTime(obj)
            time=obj.sim.simxGetLastCmdTime(obj.clientID);
        end    
        
        function set_quaternion_table(obj,x,y,z,w)
            obj.sim.simxSetObjectQuaternion(obj.clientID,obj.handle_table(1),-1,[x,y,z,w], obj.sim.simx_opmode_oneshot);	
        end    
        
        function set_position_table(obj,x,y,z) %not good
            obj.sim.simxSetObjectPosition(obj.clientID,obj.handle_table(1),-1,[x,y,z], obj.sim.simx_opmode_oneshot);	
        end    
        
        function pos = position_of_table(obj)
            	[~, pos]=obj.sim.simxGetObjectPosition(obj.clientID,obj.handle_table(1),-1,obj.sim.simx_opmode_blocking);
        end        
        
        function vel_obj = velocity_of_table(obj)
            	[~, vel_obj,~]=obj.sim.simxGetObjectVelocity(obj.clientID,obj.handle_table(1),obj.sim.simx_opmode_buffer);
        end    
        
         function contact_force = GetF(obj)
            [~,contact_force_string] = obj.sim.simxGetStringSignal(obj.clientID,'ContactForce',obj.sim.simx_opmode_buffer);
            contact_force_string;
            contact_force = obj.sim.simxUnpackFloats(contact_force_string);
%               [res,contact_force_string] = vrep.simxGetStringSignal(id,'ContactForce',vrep.simx_opmode_buffer);
%                contact_force = vrep.simxUnpackFloats(contact_force_string)
         end
         function Inter = Intergate(obj)
             Inter=1;
             obj.sim.simxSynchronousTrigger(obj.clientID);
         end
         
         function Init = Inital(obj)
             Init=1;
             dt = 5e-3;
             obj.sim.simxSetFloatingParameter(obj.clientID,obj.sim.sim_floatparam_simulation_time_step,dt,obj.sim.simx_opmode_oneshot_wait);
      % enable the synchronous mode on the client:
    obj.sim.simxSynchronous(obj.clientID,true);
    % start the simulation:
    obj.sim.simxStartSimulation(obj.clientID,obj.sim.simx_opmode_oneshot_wait);
    obj.sim.simxSynchronousTrigger(obj.clientID);
    
   for i=1:7
    obj.sim.simxGetJointPosition(obj.clientID,obj.robot_joints(i),obj.sim.simx_opmode_streaming); % joint position
    obj.sim.simxGetObjectFloatParameter(obj.clientID,obj.robot_joints(i),2012,obj.sim.simx_opmode_streaming); % joint velocity
    obj.sim.simxGetJointForce(obj.clientID,obj.robot_joints(i),obj.sim.simx_opmode_streaming); % joint torque
    obj.sim.simxGetStringSignal(obj.clientID,'ContactForce',obj.sim.simx_opmode_streaming); % contact force
   end
    
     obj.sim.simxGetPingTime(obj.clientID);
         end
         
    end
end

