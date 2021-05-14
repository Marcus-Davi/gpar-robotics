% nome = input('Digite o nome do arquivo que deseja ler: ','s');
nome = strcat('pitch_imu','.csv');

rosshutdown % desligar antes
rosinit % roscore


data = csvread(nome);
accx = data(:,4);
accy = data(:,5);
accz = data(:,6);

gyrox = data(:,1);
gyroy = data(:,2);
gyroz = data(:,3);


dt = 0.01;
size_data = length(gyrox);

angle_x = 0;
angle_y = 0;
angle_z = 0;

% Ros data
tftree = rostf;
tform = rosmessage('geometry_msgs/TransformStamped');
tform.ChildFrameId = 'imu';
tform.Header.FrameId = 'map';
tform.Transform.Translation.X = 0;
tform.Transform.Translation.Y = 0;
tform.Transform.Translation.Z = 0;
tform.Transform.Rotation.W = 1;
tform.Transform.Rotation.X = 0;
tform.Transform.Rotation.Y = 0;
tform.Transform.Rotation.Z = 0;

for i = 1 : size_data
    angle_x = angle_x + dt * gyrox(i);
    angle_y = angle_y + dt * gyroy(i);
    angle_z = angle_z + dt * gyroz(i);
    eul = [angle_x angle_y angle_z]; % XYZ roll pitch yaw
    
    quat = eul2quat(eul,'xyz');
    tform.Transform.Rotation.W = quat(1);
    tform.Transform.Rotation.X = quat(2);
    tform.Transform.Rotation.Y = quat(3);
    tform.Transform.Rotation.Z = quat(4);
    tform.Header.Stamp = rostime('now')
    sendTransform(tftree,tform)
    pause(dt)
end