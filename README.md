# Multiple face tracking in video
 
MATLAB implemented software for multi-face tracking in videos. Originally developed for an [ECCV 2016 workshop paper](#citation) to track characters' faces in TV shows.
 
Code provided by James Charles.
 
This work is licensed under the Creative Commons Attribution 4.0 International License. To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/ or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
 
## What it does
 
Given an input video the software will track face bounding boxes sequentially from one frame to the next. Tracks are saved to disk and a resume feature allows you to stop and start tracking from where you left off.
 
![Example tracking1](https://raw.githubusercontent.com/jjcharles/facetracking/master/images/frame_00050_reduced.jpg)
![Example tracking2](https://raw.githubusercontent.com/jjcharles/facetracking/master/images/frame_00138_reduced.jpg)
## Included
 
* Software for tracking faces in videos.
* Visualisation function to view the tracks.
* Demo script and video to illustrate how to use the code.

## How it works

This software detects faces using the frontal-face detector of Viola-Jones. Face bounding boxes are then tracked using the Kanade-Lucas-Tomasi (KLT) algorithm. The tracked key points (BRISK) are updated when a new face detection appears. Key points are only searched for within the centre of the face bounding box, this reduces drift to background content but also enables tracking to profile faces.
 
## Dependencies
 
The code has been tested in MATLAB R2014a and will presumably also run in later versions. 
 
The following MATLAB toolboxes are required:
 
* Computer Vision System Toolbox
* Control System Toolbox
* Image Processing Toolbox
 
## Citation
 
If you use this code then please cite:
 
```
@Article{Charles16b,
  author       = "Charles, J. and Magee, D. and Hogg.",
  title        = "Virtual Immortality: Reanimating characters from TV shows",
  booktitle    = "ECCV Workshop on Virtual/Augmented Reality for Visual Artificial Intelligence (VARVAI)",  
  year         = "2016",
}
```

