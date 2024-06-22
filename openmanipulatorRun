%%load the robot after modifying the URDF according to the end 
%effector position
filename= 'open_manipulator_camera.urdf'; 
robot1 = importrobot(filename) ;
axes = show(robot1);
axes.CameraPositionMode = 'auto';
hold on
%% Subscribe the topics used ahead
subs1 = rossubscriber("/open_manipulator/joint_states");
subs2= rossubscriber("/open_manipulator/states");
s2='STOPPED';
s2=['"' s2 '"'];
s2=string(s2);
pause(3);

%% set the robot to camera snap pose such that the test object is 
%Field of View of the camera.
errorOfPoints=[];
ik = robotics.InverseKinematics('RigidBodyTree',robot1);
weights = [0.1 0.1 0 1 1 1];
testclient = rossvcclient("/open_manipulator/goal_joint_space_path");
testreq = rosmessage(testclient);
pause(1);
testreq.PlanningGroup = "some";
testreq.JointPosition.JointName =["joint1";"joint2";"joint3";
"joint4"];
testreq.JointPosition.Position = [-0.064427;-0.662679;0.168737;
1.593806];
testreq.JointPosition.MaxAccelerationsScalingFactor = 1.0;
testreq.JointPosition.MaxVelocityScalingFactor = 1.0;
testreq.PathTime = 2.0;
[testreq.JointPosition.Position, error]= 
Pcontrol4joints(testreq.JointPosition.Position,testreq,
testclient,subs1,subs2);
waitForServer(testclient,"Timeout",3);
testresp = call(testclient,testreq,"Timeout",3);
while 1
        status= receive(subs2,10);
        if strcmp(string(status.OpenManipulatorMovingState),s2)
            break
        
        end
       
end

%% get current joint states

current_joint_states = receive(subs1,10);
inputConfig = homeConfiguration(robot1);
inputConfig(1).JointPosition = current_joint_states.Position(1);
inputConfig(2).JointPosition = current_joint_states.Position(2);
inputConfig(3).JointPosition = current_joint_states.Position(3);
inputConfig(4).JointPosition = current_joint_states.Position(4);
 
%% camera to Base transformation along with normals
xyz_trans = Camera2Base(x,y,z,robot1,inputConfig);
xB=[];yB=[];zB=[];u=[];v=[];w=[];
[xB,yB,zB,u,v,w] = createNormals(xyz_trans,robot1,inputConfig);
%% normal correction
unit_normal=[0 0 1];
uvw=[u v w];
index=[];
for i=1:size(uvw,1)
    theta = acosd(dot(unit_normal,uvw(i,:)));
    if theta<10     %reject all normals that are at an 
    %angle > 10 degress w.r.t. Z axis
        index(end+1)=i;
    end
end
xn=[];
yn=[];
zn=[];
un=[];
vn=[];
wn=[];
for i=1:length(index)
    xn((end+1),1)=xB(index(i));
    yn((end+1),1)=yB(index(i));
    zn((end+1),1)=zB(index(i));
    un((end+1),1)=u(index(i));
    vn((end+1),1)=v(index(i));
    wn((end+1),1)=w(index(i));
end
xB=xn;yB=yn;zB=zn;u=un;v=vn;w=wn;
%% visualization of points and normals
ptcloud = pointCloud([xB';yB';zB']');
pcshow(ptcloud,MarkerSize=50);
hold on
axis equal
quiver3(xB,yB,zB,u,v,w);
hold off


%% run iksolver and the robot for each point
initialguess = inputConfig;
dataAScan=[];
for i = 1: length(xB)
    tx=[xB(i) yB(i) zB(i)];
    tform = trvec2tform(tx);
    theta1 = -atan(v(i)/w(i));
    theta2 = asin(u(i));
    theta3 = 0;
    euler_angles = [theta1 theta2 theta3];
    rotat_mat = eul2rotm(euler_angles,'XYZ');
    tform(1:3,1:3)=rotat_mat;
    % Call inverse kinematics solver for every end-effector position 
    % using the previous configuration as initial guess
    idx =1;
    current_joint_states = receive(subs1,10);
    inputConfig(1).JointPosition = current_joint_states.Position(1);
    inputConfig(2).JointPosition = current_joint_states.Position(2);
    inputConfig(3).JointPosition = current_joint_states.Position(3);
    inputConfig(4).JointPosition = current_joint_states.Position(4);
    initialguess = inputConfig;
    configSoln(idx,:) = ik('end_effector_link',tform,weights,
    initialguess);
    testreq.PlanningGroup = "some";
    testreq.JointPosition.JointName =["joint1";"joint2";"joint3";
    "joint4"];
    testreq.JointPosition.Position = [configSoln(idx,1).JointPosition;
    configSoln(idx,2).JointPosition;
    configSoln(idx,3).JointPosition;
    configSoln(idx,4).JointPosition];
    testreq.JointPosition.MaxAccelerationsScalingFactor = 1.0;
    testreq.JointPosition.MaxVelocityScalingFactor = 1.0;
    testreq.PathTime = 2.0;
    %Use closed loop control function "Pcontrol4joints" to optimize the
    %joint input
    [testreq.JointPosition.Position, error]=
    Pcontrol4joints(testreq.JointPosition.Position,testreq,
    testclient,subs1,subs2);
    errorOfPoints((end+1),:)=error;
    %%
    waitForServer(testclient,"Timeout",10);
    testresp = call(testclient,testreq,"Timeout",3);
    while 1
        status= receive(subs2,10);
        if strcmp(string(status.OpenManipulatorMovingState),s2)
            break
        
        end
       
    end
    disp(i);
    pause(0.3);
    %get data from CRO
    [wave, time] = pyrunfile ("getCROData.py",["wave","time"]); 
    newWave = double(wave);
    dataAScan((end+1),:) = newWave;
    testreq.PlanningGroup = "some";
    testreq.JointPosition.JointName =["joint1";"joint2";
    "joint3";"joint4"];
    testreq.JointPosition.Position = [-0.2178;-0.2239;
    0.8099;-0.5046];
    testreq.JointPosition.MaxAccelerationsScalingFactor = 1.0;
    testreq.JointPosition.MaxVelocityScalingFactor = 1.0;
    testreq.PathTime = 2.0;
    waitForServer(testclient,"Timeout",3);
    testresp = call(testclient,testreq,"Timeout",3);
    while 1
        status= receive(subs2,10);
        if strcmp(string(status.OpenManipulatorMovingState),s2)
            break

        end

    end
    
end
