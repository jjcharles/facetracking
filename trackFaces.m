function [faceDet,faceTracks] = trackFaces( videoFilename,fittingModel,outFilename,visualise )
%TRACKFACES tracks all faces in a video
%   INPUT
%       videofilename - path to input video
%       fittingModel - model options for face detection
%       outfilename - path where we want to store our face detections
%       visualise - boolean to turn on visualisation
%   OUTPUT
%       faceDet - bounding box detections indexed by frame id
%       faceTracks - bounding box detections indexed by track id

%  define track structure (track id based)
%     faceTracks(trackID).bbox;
%     faceTracks(trackID).frameid;
    
%  define detection structure (frame id based)
%     faceDet(frameID).trackIDs;
%     faceDet(frameID).bboxes;
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

    faceTracks = [];
    faceDet = [];
    h_img = []; %for visualisation only (image handle)

    %load video object
    vidobj = VideoReader(videoFilename);
    
    %setup face detector
    faceDetector = vision.CascadeObjectDetector();
    faceDetector.MaxSize = round(fittingModel.maxsize*fittingModel.facedetect.imscale);
    faceDetector.MinSize = round(fittingModel.minsize*fittingModel.facedetect.imscale);
    faceDetector.MergeThreshold = fittingModel.facedetect.sensitivity;
    
    %setup trackers
    trackers(1).maxID = 0;
    trackers(1).numActive = 0;
    
    %get starting frame index based on current saved output and also
    %tracker info
    [startIndex,trackers] = getSavedIndex(outFilename);
    
    %load the saved tracks
    if exist(outFilename,'file');
        load(outFilename);
    end
    
    %loop over frames and track
    for i = startIndex:vidobj.NumberOfFrames
        if mod(i,1000)==0
            fprintf('frame %d of %d, number of active trackers: %d\n',i,vidobj.NumberOfFrames,trackers(1).numActive);
        end

        img = read(vidobj,i);
        
        %track all previous faces to this frame
        [faceDet,faceTracks,trackers] = track(faceDet,faceTracks,trackers,img,i);
        
        %detect all faces
        facebboxes = detectFaces(faceDetector,img);
        
        %start new tracks if we detect new faces
        [faceDet,faceTracks,trackers] = addNewFaces(faceDet, faceTracks,trackers,facebboxes,img,i);
        
        %save output every 1000 frames
        saveOutput(outFilename,faceDet,faceTracks,trackers,vidobj.NumberOfFrames,i,1000); 
        
        %visualise
        if visualise
            h_img = vis(img,faceDet,trackers,i,h_img);
        end
    end
    
    %produce tracking structure from faceDet
    if isempty(faceTracks)
        faceTracks = dets2tracks(faceDet);
        frameID = vidobj.NumberOfFrames;
        save(outFilename,'faceDet','frameID','faceTracks');
    end
end

%function to convert faceDet structure to a faceTracks structure
function faceTracks = dets2tracks(faceDet)
    faceTracks(1).bboxes = [];
    faceTracks(1).frameIDs = [];
    
    for frameID = 1:numel(faceDet)
        if ~isempty(faceDet(frameID).trackIDs)
            for t = 1:numel(faceDet(frameID).trackIDs)
                trackID = faceDet(frameID).trackIDs(t);
                if trackID > numel(faceTracks)
                    faceTracks(trackID).bboxes = faceDet(frameID).bboxes(t,:);
                    faceTracks(trackID).frameIDs = frameID;
                else
                    faceTracks(trackID).bboxes = cat(1,faceTracks(trackID).bboxes,faceDet(frameID).bboxes(t,:));
                    faceTracks(trackID).frameIDs = cat(2,faceTracks(trackID).frameIDs,frameID);
                end
            end
        end
    end
end

%function to return start index for tracking
function [index,trackers] = getSavedIndex(outFilename)
    if ~exist(outFilename,'file')
        index = 1;
        trackers(1).maxID = 0;
        trackers(1).numActive = 0;
    else
        data = load(outFilename);
        index = data.frameID + 1; 
        trackers = data.trackers;
    end
end

%function to save the tracks
function saveOutput(outFilename,faceDet,faceTracks,trackers,maxFrames,frameID,stepSize)
    if frameID == maxFrames || mod(frameID,stepSize)==0
        save(outFilename,'faceDet','faceTracks','frameID','trackers');
    end
end

%track all faces to the next frame (destroys trackers when neccessary)
function [faceDet,faceTracks,trackers] = track(faceDet,faceTracks,trackers,img,frameID)
    if trackers(1).maxID == 0 || trackers(1).numActive == 0 
        return
    else
        %for each tracker track bbox to new image
        for i = 1:numel(faceDet(frameID-1).trackIDs)
            trackerID = find(faceDet(frameID-1).trackIDs(i)==cat(1,trackers(:).trackID));  %find tracker associated with this face
            if isempty(trackerID); continue; end %if tracker destroyed then skip
            [isGood,facebbox,tracker] = trackBBox(trackers(trackerID),img,faceDet(frameID-1).bboxes(i,:)); %track the face forward in time
            
            %update faceDet and faceTracks with new data
            if isGood
                [faceDet,faceTracks] = updateFace(faceDet,faceTracks,facebbox,trackers(trackerID).trackID,frameID);
                %update the tracker
                trackers(trackerID).points = tracker.oldPoints;
                trackers(trackerID).tracker = tracker.tracker;
            else
                if numel(faceDet)<frameID
                    faceDet(frameID).bboxes = [];
                end
                %we need to remove this tracker and let the system
                %initialise a new one when it gets a good face detection
                trackers(1).numActive = max(0,trackers(1).numActive - 1);
                release(trackers(trackerID).tracker);
                
                if trackerID == 1
                    if numel(trackers)==1
                        trackers(trackerID).points = [];
                        trackers(trackerID).tracker = [];
                    else
                        maxID = trackers(1).maxID;
                        numActive = trackers(1).numActive;
                        trackers(trackerID) = [];
                        trackers(trackerID).maxID = maxID;
                        trackers(trackerID).numActive = numActive;
                    end
                else
                    trackers(trackerID) = [];
                end
            end
        end
    end
end

function [isGood, facebbox, tracker] = trackBBox(tracker,img,facebbox)
    isGood = false;
    tracker.oldpoints = tracker.points;
    [points, isFound] = step(tracker.tracker, img);
    tracker.points = points(isFound, :);
    oldInliers = tracker.oldpoints(isFound, :);
            
    if size(tracker.points, 1) >= 3 % need at least 4 points
        % Estimate the geometric transformation between the old points
        % and the new points and eliminate outliers
        [xform, oldInliers, visiblePoints] = estimateGeometricTransform(...
            oldInliers, tracker.points, 'similarity', 'MaxDistance', 5);

        % Position bounding box at centre of points
        centre = mean(tracker.points);
        tempFacebbox = [centre(1)-facebbox(3)/2,centre(2)-facebbox(4)/2,facebbox(3),facebbox(4)];
        tempFacebbox = round(tempFacebbox);

        %check if tempFacebbox very different, if so track is not good
        A1 = prod(facebbox(3:4)); %area of new bounding box
        A2 = prod(tempFacebbox(3:4)); %area of old bounding box
        %compute intersection area
        U = rectint(facebbox,tempFacebbox);
            
        %calculate a similarity score and threshold at 0.7
        similarity = (U/A1 + U/A2)/2;
        if similarity > 0.7
            isGood = true;
        else
            isGood = false;
        end
            
        facebbox = tempFacebbox;
        tracker.oldPoints = visiblePoints;
        setPoints(tracker.tracker, tracker.oldPoints);
    end
end

function [faceDet,faceTracks,trackers] = addNewFaces(faceDet, faceTracks,trackers,facebboxes,img,frameID)
    %for each box see if it is a new face
    if ~isempty(facebboxes)
        numDets = size(facebboxes,1);
        for i = 1:numDets
            if ~isempty(faceDet)
                [isNew,matchingFaceID] = isNewFace(faceDet,facebboxes(i,:),frameID);
                if isNew
                    [faceDet,faceTracks,trackers] = addFace(faceDet,faceTracks,trackers,facebboxes(i,:),img,frameID);
                else
                    %update tracker with points in detected face
                    if ~isempty(matchingFaceID)
                        trackerID = find(cat(1,trackers(:).trackID)==faceDet(frameID).trackIDs(matchingFaceID));
                        if ~isempty(trackerID)
                            %re-initialise the tracker
                            trackers(trackerID).points = cat(1,trackers(trackerID).points,getFeaturePoints(img,facebboxes(i,:)));

                            %randomly sample at most 30 points
                            Ridx = randperm(size(trackers(trackerID).points,1));
                            trackers(trackerID).points = trackers(trackerID).points(Ridx(1:min(numel(Ridx),30)),:);
                            setPoints(trackers(trackerID).tracker, trackers(trackerID).points);
                        end
                    end
                end
            else
                [faceDet,faceTracks,trackers] = addFace(faceDet,faceTracks,trackers,facebboxes(i,:),img,frameID);
            end
        end
    end
end

%this function adds a new face and initialises a new tracker
function [faceDet,faceTracks,trackers] = addFace(faceDet,faceTracks,trackers,facebbox,img,frameID)
    numActiveTrackers = trackers(1).numActive;
    
    %get features
    points = getFeaturePoints(img,facebbox);
    
    if ~isempty(points)
        %setup new tracker
        trackerID = numActiveTrackers+1;
        trackers(1).maxID = trackers(1).maxID + 1; %maxID stores the total number of trackers so far
        trackers(1).numActive = trackers(1).numActive + 1;
        trackers(trackerID).trackID = trackers(1).maxID;
        trackers(trackerID).tracker = vision.PointTracker('MaxBidirectionalError', 2);

        %get features to track
        trackers(trackerID).points = points;

        %initialise the tracker
        initialize(trackers(trackerID).tracker,trackers(trackerID).points,img);

        %update the faceDet and faceTracks structure
        [faceDet,faceTracks] = updateFace(faceDet,faceTracks,facebbox,trackers(trackerID).trackID,frameID);
    end
end

%this function updates the faceDet and faceTracks structure with a new
%bounding box
function [faceDet,faceTracks] = updateFace(faceDet,faceTracks,facebbox,trackID,frameID)

    if ~isempty(faceDet)
        if numel(faceDet)<frameID % add new face and frame
            faceDet(frameID).trackIDs = trackID;
            faceDet(frameID).bboxes = facebbox;
            faceDet(frameID).landmarks = [];
        else %append face to current frame
            faceDet(frameID).trackIDs = cat(2,faceDet(end).trackIDs,trackID);
            faceDet(frameID).bboxes = cat(1,faceDet(end).bboxes,facebbox);
            faceDet(frameID).landmarks = [];
        end
    else
        faceDet(frameID).trackIDs = trackID;
        faceDet(frameID).bboxes = facebbox;
        faceDet(frameID).landmarks = [];
    end

end

function bboxes = detectFaces(faceDetector,img)
    bboxes = step(faceDetector, img);   
end

%check if the current bounding box is a new detection
function [newface,matchingFaceID] = isNewFace(faceDet,facebbox,frameID)
    newface = false;
    matchingFaceID = [];
    if isempty(faceDet)
        newface = true;
    elseif numel(faceDet)<frameID
        newface = true;
    else
        %check overlap of facebbox with bounding boxes present in faceDet
        numBBoxes = size(faceDet(frameID).bboxes,1);
        A1 = prod(facebbox(3:4)); %area of new bounding box
        for i = 1:numBBoxes
            A2 = prod(faceDet(frameID).bboxes(i,3:4)); %area of old bounding box
            %compute intersection area
            U = rectint(facebbox,faceDet(frameID).bboxes(i,:));
            
            %calculate a similarity score and threshold at 0.9
            similarity = (U/A1 + U/A2)/2;
            if similarity > 0.6
                newface = false;
                matchingFaceID = i;
                break;
            else
                newface = true;
            end
        end
    end
end

%function to convert a bounding box to points (unused)
function points = bbox2point(bbox)
    points = zeros(4,2);
    points(1,:) = bbox(1:2);
    points(2,:) = points(1,:) + [0 bbox(3)];
    points(3,:) = points(2,:) + [bbox(4) 0];
    points(4,:) = points(1,:) + [bbox(4) 0];
end

%function to convert points to a bounding box (unused)
function bbox = point2bbox(points)
    bbox = zeros(1,4);
    bbox(1) = min(points(:,1));
    bbox(2) = min(points(:,2));
    bbox(3) = max(points(:,1))-bbox(1);
    bbox(4) = max(points(:,2))-bbox(2);
end

%function to return feature points given bounding box and image
function locations = getFeaturePoints(img,bbox)
%     points = detectMinEigenFeatures(rgb2gray(img), 'ROI', bbox);
    points = detectBRISKFeatures(rgb2gray(img), 'ROI', bbox,'MinContrast',0.001);
    locations = points.Location;
    if ~isempty(locations)

        %get cluster close to centre of points
        centre = mean(locations);
        dist = sqrt(sum(bsxfun(@minus,locations,centre).^2,2));
        i = 1;
        idrem = ones(1,size(locations,1));
        while sum(idrem)>=size(locations,1)
            idrem = dist>15*i;
            i = i + 0.5;
        end
        locations(idrem,:) = [];
    end
end

%function to visualise the output
function h_img = vis(img,faceDet,trackers,frameID,h_img)
    if isempty(h_img) %create figure and image pane
        figure
        h_img = imagesc(img); axis image;
    end
    
    if numel(faceDet)>=frameID
        for b = 1:size(faceDet(frameID).bboxes,1)
            img = insertObjectAnnotation(img,'rectangle',faceDet(frameID).bboxes(b,:),sprintf('track: %d',faceDet(frameID).trackIDs(b)));
        end
        for b = 1:size(faceDet(frameID).bboxes,1)
            trackerID = find(cat(1,trackers(:).trackID)==faceDet(frameID).trackIDs(b));
            if ~isempty(trackerID)
                plotPoints = cat(2,trackers(trackerID).points(:,1:2),2*ones(size(trackers(trackerID).points,1),1));
                img = insertObjectAnnotation(img,'circle',plotPoints,' ','TextBoxOpacity',0,'Color','white');
            end
        end

    end
    set(h_img,'cdata',img);
    drawnow  
end
