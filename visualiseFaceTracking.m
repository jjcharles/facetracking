function visualiseFaceTracking( videoFilename,trackingFilename,trackID )
%VISUALISEFACETRACKING visualise the face tracking
%   INPUT
%   videoFilename - filename of video
%   trackingFilename - filename to mat file containing faceDet structure
%
%   Written by James Charles at University of Leeds 27/04/16
%
%   This work is licensed under the Creative Commons Attribution 4.0 International License. 
%   To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/ or 
%   send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

    vidobj = VideoReader(videoFilename);
    if ~exist('trackID','var')
        load(trackingFilename);
        h_img = [];
        for i = 1:vidobj.NumberOfFrames
            img = read(vidobj,i);
            h_img = vis(img,faceDet,i,h_img);
        end
    else
        load(trackingFilename);
        h_img = [];
        for f = faceTracks(trackID).frameIDs
            id = find(faceDet(f).trackIDs==trackID);
            faceDet(f).bboxes = faceDet(f).bboxes(id,:);
            faceDet(f).trackIDs = faceDet(f).trackIDs(id);
        end
            
        for i = faceTracks(trackID).frameIDs
            img = read(vidobj,i);
            h_img = vis(img,faceDet,i,h_img);
        end
    end
end

%function to visualise the output
function h_img = vis(img,faceDet,frameID,h_img)
    if isempty(h_img) %create figure and image pane
        figure
        h_img = imagesc(img); axis image;
    end
    
    if numel(faceDet)>=frameID
        for b = 1:size(faceDet(frameID).bboxes,1)
            img = insertObjectAnnotation(img,'rectangle',faceDet(frameID).bboxes(b,:),sprintf('track: %d',faceDet(frameID).trackIDs(b)));
        end
    end
    set(h_img,'cdata',img);
    drawnow
    
    %output frames for a video
%     videoFolder = check_dir('./working_dir/video_frames_v5/',true);
%     filename = sprintf('%sframe_%05d.jpg',videoFolder,vidCount);
%     vidCount = vidCount + 1;
%     imwrite(img,filename);     
end

