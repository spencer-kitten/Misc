%Cross Track Error example
clear all
close all
clc

% Initialize working variables
x=10;
y=30;
jj=1;
rho=10;
u=1.5;
v=0;
desiredRate = 1; %original 1 hertz - one time a second
rate = rosrate(desiredRate);
REF_DEPTH = 10.0;
MAX_PROP = 1200; %maximum rpms limits [1200,-1200]
REF_HEAD = 90; %degrees

% Define Track
X1=0;Y1=0;
X2=100;Y2=100;
X3=100;Y3=300;
X4 = 300; Y4 = 300;
X5 = 300;Y5 = 100;
X6 = 100;Y6 = 100;
X=[X1,X2,X3,X4,X5,X6];Y=[Y1,Y2,Y3,Y4,Y5,Y6];

% Intitialize transverse matrix
T=zeros(2,1);

% Plot initial track
figure(1), plot(X,Y,'rs');
axis([0 400 0 400])
hold on
figure(1), plot(X,Y,'ro','MarkerSize',25);
line(X,Y)
figure(1), plot(x,y,'b');
title('Cross Track Error Example')
xlabel('x');
ylabel('y');

%creates the publish object
bow_port_thruster_pub = rospublisher('/bow_port_thruster', 'std_msgs/Float64');
bow_stbd_thruster_pub = rospublisher('/bow_stbd_thruster', 'std_msgs/Float64');
vert_port_thruster_pub = rospublisher('/vert_port_thruster','std_msgs/Float64');
vert_stbd_thruster_pub = rospublisher('/vert_stbd_thruster','std_msgs/Float64');
aft_port_thruster_pub = rospublisher('/aft_port_thruster', 'std_msgs/Float64');
aft_stbd_thruster_pub = rospublisher('/aft_stbd_thruster', 'std_msgs/Float64');
aft_vert_thruster_pub = rospublisher('/aft_vert_thruster', 'std_msgs/Float64');

%create the publish message
bow_port_thruster_msg = rosmessage(bow_port_thruster_pub);
bow_stbd_thruster_msg = rosmessage(bow_stbd_thruster_pub);
vert_port_thruster_msg = rosmessage(vert_port_thruster_pub);
vert_stbd_thruster_msg = rosmessage(vert_stbd_thruster_pub);
aft_port_thruster_msg = rosmessage(aft_port_thruster_pub);
aft_stbd_thruster_msg = rosmessage(aft_stbd_thruster_pub);
aft_vert_thruster_msg = rosmessage(aft_vert_thruster_pub);

%subscribe to gazebo odometry
fusion_state_sub = rossubscriber('/fusion/pose_gt');

x = [];
not_received_odometry = 1;
k=0;

while(not_received_odometry)
    k=k+1
    statedata = receive(fusion_state_sub,3);
    posit = statedata.Pose.Pose.Position;
    x = posit.X
    y = posit.Y
    z = posit.Z

    if (~isempty(x))
        not_received_odometry = 0;
    end
    waitfor(rate);
    disp('waiting for odometry');
end

disp('received odometry');

% Initialize depth error working variables
depth_err_cum = 0;
depth_err_old = 0;
depth_err1 = 0;
depthdata=0;

% Initialize heading error working variables
head_err_cum = 0;
head_err_old = 0;
head_err1 = 0;
head_data=0;

%Initialize plotting variables
time=0;

%Plotting Indexes
i=0;
j=0;

%Plotting variables depth error
a=0;
b=0;
c=0;

%Plotting variables heading error
Prop_head_err = 0;
Int_head_err  = 0;
Der_Head_err  = 0;


while (true)
    %Pull data from simulation
    statedata = receive(fusion_state_sub,3);
    posit = statedata.Pose.Pose.Position;
    x = posit.X;
    y = posit.Y;
    depth = posit.Z;
    quat = statedata.Pose.Pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    Theta = rad2deg(angles(2));
	  head = rad2deg(angles(1));

    %Modify plotting working varaibles and index
    i=i+1;
    time(i)=i;

    %remember negative is down in Gazebo

    %if for heading correection
%     if (head<0)
%         head=head+360;
%     end

    %code for the PID controller goes here: goodish gains, 120:2:10
    %better gains? 150:2:15

    %500::5:15

    %Depth Error
    K_p=600;
    K_i=5;
    K_d=20;

    %Update current depth error and cumulative depth error
    depth_err1 = -REF_DEPTH - depth
    depth_err_cum = depth_err_cum + depth_err1 %code for the PID controller goes here

    %changed deriv equation to old-err1/old, removed 170 summation
    PID =(K_p*depth_err1 + K_i*depth_err_cum + K_d*(depth_err_old-depth_err1)/depth_err_old);

    %Cap possible PID signal
    if (PID>1200)
        PID=1200;
    elseif(PID<-1200)
        PID=-1200;
    end

    %Plotting Depth Error Statistics
    PIDa(time(i))=PID;
    a(time(i))=K_p*depth_err1;
    b(time(i))=K_i*depth_err_cum;
    c(time(i))=K_d*(depth_err_old-depth_err1)/depth_err1;

    gain=1;

    %send the message here - the send function has two arguments the
    vert_stbd_thruster_msg.Data=(gain*PID);
    vert_port_thruster_msg.Data=(-gain*PID);
    %aft_stbd_thruster_msg.Data=20;
    %bow_stbd_thruster_msg.Data=2;

    %publisher and the message
    send(vert_stbd_thruster_pub, vert_stbd_thruster_msg);
    send(vert_port_thruster_pub, vert_port_thruster_msg);
    %send(aft_stbd_thruster_pub, aft_stbd_thruster_msg);
    %send(bow_stbd_thruster_pub, bow_stbd_thruster_msg);

    %Update working variables and plotting variables
    depth_err_old=depth_err1;
    depthdata(time(i))=depth;

    %%Heading Error 60:5:2
    K_p=5;
    K_i=1;
    K_d=1;

    head_err1 = wrapTo2Pi(deg2rad(REF_HEAD)) - wrapTo2Pi(head);

    head_err_cum = head_err_cum + head_err1;

    %Calculate PID error for heading
    PID_head =(K_p*head_err1 + K_i*head_err_cum + K_d*(head_err_old-head_err1)/head_err_old);
    %PID_head= -PID_head;
    if (PID_head>1200)
        PID_head=1200;
    elseif (PID_head<-1200)
        PID_head=-1200;
    end

    %Update settings and publish for bow thrusters
    bow_stbd_thruster_msg.Data=(1.5*PID_head);
    bow_port_thruster_msg.Data=(1.5*PID_head);
    send(bow_stbd_thruster_pub, bow_stbd_thruster_msg);
    send(bow_port_thruster_pub, bow_port_thruster_msg);

    %Update settings and publish for aft thrusters
    %-200 and 200 stbd/port seems to work
    aft_port_thruster_msg.Data=(250);
    aft_stbd_thruster_msg.Data=(-250);

    send(aft_port_thruster_pub, aft_port_thruster_msg);
    send(aft_stbd_thruster_pub, aft_stbd_thruster_msg);
    %End Heading Error

    % Code to calculate cross track error
    T=[(X(jj+1)-X(jj));(Y(jj+1)-Y(jj))];
    N=[(Y(jj+1)-Y(jj));-(X(jj+1)-X(jj))];
    P=[(x-X(jj));(y-Y(jj))];
    e=P'*N/sqrt(N'*N)
    %distance from waypoint
    s1=P'*T/sqrt(T'*T)
    %psitrack=atan2((Y(jj+1)-Y(jj)),(X(jj+1)-X(jj)));
    if(abs(s1) > 0.90*sqrt(T'*T))
        jj=jj+1;
    end

    REF_HEAD = rad2deg(psitrack + atan2(e,rho));

    %Plotting variables for heading error
    Prop_head_err(time(i)) = K_p*head_err1;
    Int_head_err(time(i))  = K_i*head_err_cum;
    Der_Head_err(time(i))  = K_d*(head_err_old-head_err1)/head_err1;
    head_data(time(i)) = head;
    head_err_old = head_err1;

    figure(1)
    plot(x,y,'b*');

    waitfor(rate);
end
