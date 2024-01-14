 %% Group 2: Camila Pazos, Sarah Fahmi, Madeleine Drefke

% This is a script that will read a video file in the form of an mp4
% located by a file name and use centroids and motion analysis to determine
% the acceleration (or deceleration) of a moving object

% function [meanAcc] = videoReader(file, frameRate) 
% TO EXECUTE THIS FUNCTION:
% Type "videoReader(xxxx, yyyy) in the command window with xxxx replaced
% with the name of an mp4 video file, and yyyy replaced with an integer.
% 
% INPUTS:
% File: This is a character string that identifies a video in the file that
% this script is saved in. E.g.: 'cropped.mp4' 
% frameRate: This is an integer that stands for the number of frames per 
% second of video. E.g.: 30

% OUTPUT: 
% meanAcc is the average acceleration of the identified object as found by
% the image processing functions.

%%
% file = 'cropped.mp4';
frameRate = 30; %30 frames/s

% Create an object that matlab will flip through frame by frame
vidObj = VideoReader('Trial 4.MP4');

% Designate a time at which we will begin collecting kinematic data
vidObj.CurrentTime = 2;

% Open the first frame, turn it to gray, then binarize it so that the
% darkest spots (those that contain the object we're tracking) are purely
% black, and all other spots are white.

frame1 = readFrame(vidObj);
gray1 = rgb2gray(frame1);
binary1 = imbinarize(gray1, 'global');

% Display the first frame (in color) so that we can draw a precise
% rectangle around an object of known dimensions so that we may estimate
% the total meters dropped by the object
imshow(frame1);
controlRect = getrect
controlHeight = 12; %INCHES
controlMeters = convlength(controlHeight, 'in', 'm');
pixel2m = controlRect(4)/controlMeters;

% Use the binary image to create a list of "corner points" which will be
% used to help the computer understand how an object moves by effectively
% finding identifiers and create a point tracker
points = detectMinEigenFeatures(binary1);
pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
initialize(pointTracker, points.Location, frame1);

% Declare and initialize matrices for velocity, acceleration, and centroid
% of the body
v = zeros(1,2);
a = zeros(1,2);centroids = [0,0];

% Iterate through the frames of the video and turn each image into a binary
% image so that we can track the movement of points and effectively the
% body throughout its descent
while hasFrame(vidObj)
    currentFrame = readFrame(vidObj);
    gray = rgb2gray(currentFrame);
    binaryImage = imbinarize(gray,'global');
    imshow(binaryImage);
   
    [trackedPoints, isFound] = step(pointTracker, currentFrame);
    visiblePoints = trackedPoints(isFound,:);
    setPoints(pointTracker, visiblePoints);
    if hasFrame(vidObj)
         currentFrame = readFrame(vidObj);
    end
  
    centroid = mean(trackedPoints(isFound, :));
    centroids = vertcat(centroids, centroid);

end


centroids(:,2) = -centroids(:,2); %flip sign of y so that more positive y is up
dims = size(centroids);
centroidM = centroids./pixel2m;

% Calculate velocity and accelerations as changes of position and velocity
% respectively with respect to time
vel = 0;
for i = 2:dims(1)
    v(i,:) = (centroidM(i,:)-centroidM(i-1,:))*frameRate;
    vel = vertcat(vel, sqrt(v(i,1).^2+v(i,2).^2));
end
acc = 0;
for i = 2:dims(1)
    a(i,:) = (v(i,:)-v(i-1,:))*frameRate;
    acc = vertcat(acc, sqrt(a(i,1).^2+a(i,2).^2));
end

% find the average acceleration using frames that correspond with freefall
% where the entire body is in view
meanAcc = mean(acc(6:end-2));
plot(acc(6:end-2));
scatter(centroidM(:,1), centroidM(:,2));
% end

