% Three rigid bodies + two wheel joints. Geometry is in metres, inertia in kg*m^2.
here = fileparts(mfilename('fullpath'));
urdfFile = fullfile(here,'..','通用URDF','urdf','balance_car.urdf');
meshDir = fullfile(here,'..','通用URDF','meshes');
robot = importrobot(urdfFile,'MeshPath',meshDir,'DataFormat','column');
robot.Gravity = [0 0 -9.80665];
figure('Name','Balance car final'); show(robot,'Visuals','on','Collisions','off'); axis equal; view(135,20);
% Robot is fixed at the root in rigidBodyTree; this preview is not a floating-base contact simulation.
% To create a Simscape Multibody model (requires its toolbox), run:
% addpath(meshDir); smimport(urdfFile,'ModelName','balance_car_imported');
% Then configure a floating base, gravity, wheel-ground contacts and the controller.
