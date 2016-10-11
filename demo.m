% Demo script for face tracking in videos
%
%   Written by James Charles at University of Leeds 27/04/16
%
%   If you use it then please cite the paper (for which is was developed): 
%     @Article{Charles16b,
%       author       = "Charles, J. and Magee, D. and Hogg.",
%       title        = "Virtual Immortality: Reanimating characters from TV shows",
%       booktitle    = "ECCV Workshop on Virtual/Augmented Reality for Visual Artificial Intelligence (VARVAI)",	
%       year         = "2016",
%     }
%
%   This work is licensed under the Creative Commons Attribution 4.0 International License. 
%   To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/ or 
%   send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

%load example video
inputFile = './demo_data/demo.avi';
vidobj = VideoReader(inputFile);

%set tracking output file
trackFile = './demo_data/demo_tracks.mat';

%set some options, like max and min detection windows
model.maxsize = [250 250]; %max size of detected face
model.minsize = [70 70]; %min size of detected face
model.facedetect.imscale =1; %run at full resolution 
model.facedetect.sensitivity = 10; %lower numbers detect more faces


%track the video and visualise while tracking. Set visualise = false to
%turn off visualisation while tracking.
visualise = true;
[faceDet,faceTracks] = trackFaces(inputFile,model,trackFile,visualise);

%Note: you can also resume tracking if it becomes interupted for some
%reason by re-running the above function. Tracking is saved every 1000
%frames by default.

%visualise precomputed tracks
visualiseFaceTracking(inputFile,trackFile);