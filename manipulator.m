clear all
close all
 
disp(newline)
disp('2D HALF DONUT MANIPULATOR')
disp(newline)
disp('NOTE: At any point if you want to exit the program, enter E (in single quotes).')
disp(newline)
 
 
f1=figure; %for changes in variables w.r.t time
f2=figure; %for robot and path animation
 
%Serial Link(RP) using modified DH parameters
 
L2=1.5; %prismatic joint offset (in cm)
 
L(1)=Link('d', 0,'a', 0, 'alpha', 0,'qlim', [0, pi], 'modified');
L(2)=Link( 'theta', 0,'a', 0, 'alpha', -pi/2, 'qlim', [0 1],'offset', L2, 'modified');
disp('Modified DH Parameters Table for RP robot')
r=SerialLink(L)
 
%forward kinematics symbolically
syms TH1 D2
disp('The Transformation matrix is given by: ')
T=r.fkine([TH1 D2])
 
%Calculation of the workspace
th1=[0:0.01:pi]; %resolution=0.01 (angle is in radians)
d2=[0:1/314:1]; %resolution=0.0032 (in cm)
[TH1_,D2_]=meshgrid(th1,d2);
%Not using 'subs' command since it increases execution time significantly
W_X=-sin(TH1_).*(L2+D2_);
W_Y=cos(TH1_).*(L2+D2_);
W_Z=zeros(size(TH1_));
%plotting workspace
figure(f2);
plot3(W_X(:),W_Y(:),W_Z(:), '.', 'Color', 'g', 'MarkerSize', 1);
 
 
%Calculation of Singularities
disp('The Jacobian in world frame (base frame) is given by:')
J0=r.jacob0([TH1 D2], 'trans')
disp('its Determinant is given by:')
det_J0 = det(J0(1:2, 1:2))
disp ('Singularity will exist at ')
D2 = solve (det_J0 == 0, D2)
disp('This value does not lie in the range of d2 and thus, there will be no singularities in the workspace ')
disp(newline)
 
 
%Homing point
disp('The Homing point of the Manipulator is:')
disp('In Joint Space: [pi/2, 0.5]')
disp('In Cartesian Space: [-0.2, 0]')
disp(newline)
Q0=[pi/2 0.5];
 
countTime=0;
while 1
    
    %make the last point as initial point for this path
    if ~(countTime==0)
        disp(newline)
        disp('Initial Point Joint Values:')
        Q0=Q1
    end
    
    %Input loop
    while 1
        
        %Input from the user
        in=input('Please Enter Desired Cartesian Co-ordinates as [x, y]: ');
        
        %Check if the user wants to exit the program
        if in == 'E'
            break
        end
        
        %Inverse Kinematics of entered point
        q1=atan2(-in(1),in(2));
        q2=(sqrt(in(1)^2 + in(2)^2))-1.5;
 
        %Check if the entered point lies in the workspace
        if ~((q1>=0 && q1<=pi)&&(q2>=0 && q2 <=1))
            disp(newline)
            disp('Point does not lie in the workspace')
            disp('Please try another point')
            disp(newline)
        else
            break
        end
    end
    
    %check if user wants to exit the program
    if in == 'E'
        break;
    end
    
    disp(newline)
    
    % Goal point
    disp('Goal Point Joint Values:')
    Q1=[q1 q2]
 
    %Trajectory selection by the user
    disp('Which Profile would you like to use for the generation of trajectory?')
    func_check=input('Enter 0 for polynomial of degree 5 and 1 for trapezoidal: ');
    
    if func_check == 'E' %check if user wants to exit the program
        break
    elseif func_check == 0
        func=@tpoly;
    elseif func_check == 1
        func=@lspb;
    end
 
    %trajectory generation (in joint space) between the points.
    [q v a]=mtraj(func, Q0, Q1, 50); %number of steps=50
 
    %Plots how the joint's postion, velocity and acceleration changes with time
    %in joint space
    figure(f1);
    subplot(3,3,1)
    plot(q(:,1))
    title('Changes in Joint 1 w.r.t time')
    ylabel('Theta 1')
 
    subplot(3,3,2)
    plot(v(:,1))
    title('Changes in Velocity of Joint 1 w.r.t time')
    ylabel('Velocity of Theta 1')
 
    subplot(3,3,3)
    plot(a(:,1))
    title('Changes in Accerlation of Joint 1 w.r.t time')
    ylabel('Acceleration of Theta 1')
 
    subplot(3,3,4)
    plot(q(:,2))
    title('Changes in Joint 2 w.r.t time')
    ylabel('d2')
 
    subplot(3,3,5)
    plot(v(:,2))
    title('Changes in Velocity of Joint 2 w.r.t time')
    ylabel('Velocity of d2')
 
    subplot(3,3,6)
    plot(a(:,2))
    title('Changes in Acceleration of Joint 1 w.r.t time')
    ylabel('Acceleration of d2')
 
 
    %Calculation of path in Cartesian Space
    fk=r.fkine(q);
    for i=1:1:50
        X(i)=fk(1,i).t(1);
        Y(i)=fk(1,i).t(2);
        Z(i)=fk(1,i).t(3);
    end
 
    %plots how X,Y,Z changes w.r.t time as the manipulator moves from one
    %configuration to the other
    subplot(3,3,7)
    plot(X)
    title('Changes in X w.r.t time')    
    ylabel('X')
 
    subplot(3,3,8)
    plot(Y)
    title('Changes in Y w.r.t time')
    ylabel('Y')
 
    subplot(3,3,9)
    plot(Z)
    title('Changes in Z w.r.t time')
    ylabel('Z')
 
    figure(f2)
    hold on
 
    %plots the trajectory followed by the manipulator in Cartesian Space.
    plot3(X,Y,Z, '.', 'Color', 'b')
 
    %displays the coordinates (Cartesian Space) on the graph
    text(X(1),Y(1), ['(' num2str(X(1)) ',' num2str(Y(1)) ')'], 'Color','b')
    text(X(end),Y(end), ['(' num2str(X(end)) ',' num2str(Y(end)) ')'],'Color','b')
 
    %produces an animation of how robot moves from one configuration to another
    %by changing its joint angles
    r.plot(q, 'workspace', [-3 0 -3 3 0 1], 'basecolor', 'w', 'notiles')
    
    countTime=countTime+1;
    
    hold off
    
end
 
%Total Time Calculation
if countTime ~=0
    disp(newline)
    disp('Time taken by manipulator to move from one interpolated point to another is 0.02s.')
    disp('As number of interpolated points are 50, so for any two user defined points, total time is 1s.')
    disp('And the total time taken (in seconds) by manipulator from Homing point to the last user defined point is: ')
    disp(countTime)
end


