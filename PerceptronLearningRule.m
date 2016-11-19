%demonstration of the Perceptron learning rule
clear;clc; close all

%% Generate the input vectors and their expected outputs
% these will be used to train the single-neuron perceptron.
% Classification boundary must be linear!

% pick an arbitrary 3D plane as classification boundary
    % create anonymous function and plot the plane
boundaryPlane = @(X, Y) 1*(X)-2*(Y)+4;
[Xmesh,Ymesh] = meshgrid(-10:0.5:10);
Zmesh = boundaryPlane(Xmesh, Ymesh);
figure
mesh(Xmesh,Ymesh,Zmesh)

%% generate random sample points to train the perceptron with
sampleCount = 5000;% more training data gives better predictions...usually
trainingData.XYZ = 8-(8+8)*rand(3,sampleCount);
% point on boundary plane that x,y components of sample correspond to:
Zhat = boundaryPlane(trainingData.XYZ(1,:), trainingData.XYZ(2,:));
% data is classified as (1) on/above the plane or (0) below the plane
trainingData.target = Zhat <= trainingData.XYZ(3,:);

% separate coordinates of test points by classification before plotting
redIndex = 1;
blueIndex = 1;
% there's a slightly more efficient way of doing this that involves
    % preallocating one of the arrays.
for i = 1:sampleCount
    if trainingData.target(i) == 1
       scatterRed1(:,redIndex) = trainingData.XYZ(:,i);
       redIndex = redIndex + 1;
    else
       scatterBlue1(:,blueIndex) = trainingData.XYZ(:,i);
       blueIndex = blueIndex + 1;
    end
end
hold on
scatter3(scatterRed1(1,:),scatterRed1(2,:),scatterRed1(3,:),'r.');
scatter3(scatterBlue1(1,:),scatterBlue1(2,:),scatterBlue1(3,:),'b.');
hold off

axis([-10,10,-10,10,-10,10])
title('Training Data and Boundary Plane')
xlabel('x->')
ylabel('y->')
zlabel('z->')
grid on

%% Train the perceptron model
% initialize W and b to zero or a small random value
W = zeros(1,3);%1 neuron by 3 inputs
b = 0;
for i = 1:sampleCount
    p = trainingData.XYZ(:,i);
    t = trainingData.target(i);
    a = perceptron(W,p,b);
    e = t-a; % error between target and output for given sample
    
    %adjust weights and bias
    W = W + e*p';
    b = b + e;
end

%% Test trained model with random points and plot classified test points
testPoints = 8-(8+8)*rand(3,sampleCount);

% get boolean vector of classified test points
Zhat = boundaryPlane(testPoints(1,:), testPoints(2,:));
testTarget = Zhat <= testPoints(3,:);

% separate and plot the classified test points
figure
testActual = logical(zeros(1,sampleCount));
redIndex = 1;
blueIndex = 1;
for i = 1:sampleCount
    if perceptron(W,testPoints(:,i),b)==1
       scatterRed2(:,redIndex) = testPoints(:,i);
       redIndex = redIndex + 1;
       testActual(1,i)=true;
    else
       scatterBlue2(:,blueIndex) = testPoints(:,i);
       blueIndex = blueIndex + 1;
       testActual(1,i)=false;
    end
end
hold on
scatter3(scatterRed2(1,:),scatterRed2(2,:),scatterRed2(3,:),'r.');
scatter3(scatterBlue2(1,:),scatterBlue2(2,:),scatterBlue2(3,:),'b.');
hold off

axis([-10,10,-10,10,-10,10])
title('Perceptron-Classified Test Points and Decision Boundary')
xlabel('x->')
ylabel('y->')
zlabel('z->')
grid on

%% compare testActual and testTarget to find errors
errLogical = testActual ~= testTarget;
totalErrs = sum(errLogical);
percentError = totalErrs/sampleCount*100;
fprintf('Classification error: %.4f%%\n',percentError);

%% plot decision boundary after training

[Xmesh,Ymesh] = meshgrid(-10:0.5:10);
% Wp+b=0, p=[x,y,z]'. do the math to solve for z.
Zmesh = (-W(1)*Xmesh-W(2)*Ymesh-b)/W(3);
hold on
mesh(Xmesh,Ymesh,Zmesh)
hold off