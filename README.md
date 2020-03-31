# LiDARTag
## Overview
This is a package for LiDARTag, described in paper: **LiDARTag: A Real-Time Fiducial Tag using Point Clouds** ([PDF](https://github.com/UMich-BipedLab/LiDARTag/blob/release_v0/LiDARTag.pdf)).

Fiducial markers are widely used in navigation as well as vision-based simultaneous localization and mapping (SLAM). They often consist of a payload, a pattern that makes each marker uniquely distinguishable. Therefore, as an aid in the automatic identification of targets in camera and LiDAR data, we build and use fiducial markers that can be identified by both a LiDAR and a camera; we called this type of fiducial marker, **LiDARTag**.

* Author: Jiunn-Kai Huang, Maani Ghaffari, Ross Hartley, Lu Gan, Ryan M. Eustice, and Jessy W. Grizzle
* Maintainer: [Bruce JK Huang](https://www.brucerobot.com/), brucejkh[at]gmail.com
* Affiliation: [The Biped Lab](https://www.biped.solutions/), the University of Michigan

This package has been tested under [ROS] Kinetic and Ubuntu 16.04.  
**[Note]** More detailed introduction will be updated shortly. Sorry for the inconvenient!

## Abstract
Image-based fiducial markers are widely used in robotics and computer vision problems such as object tracking in cluttered or textureless environments, camera (and multi-sensor) calibration tasks, or vision-based simultaneous localization and mapping (SLAM). The state-of-the-art fiducial marker detection algorithms rely on consistency of the ambient lighting. This paper introduces LiDARTag, a novel fiducial tag design and detection algorithm suitable for light detection and ranging LiDAR point clouds. The proposed tag runs in real-time and can process data faster than the currently available LiDAR sensors frequencies. Due to the nature of the LiDAR's sensor, rapidly changing ambient lighting will not affect detection of a LiDARTag; hence, the proposed fiducial marker can operate in a completely dark environment. In addition, the LiDARTag nicely complements available visual fiducial markers as the tag design is compatible with available techniques, such as AprilTags, allowing for efficient multi-sensor fusion and calibration tasks. The experimental results, verified by a motion capture system, confirm the proposed technique can reliably provide a tag's pose and its unique ID code. All implementations are done in C++ and available at: https://github.com/UMich-BipedLab/LiDARTag

## Quick View
<img src="https://github.com/brucejk/LiDARTag/blob/master/figure/figure1-2.png" width="640">
<img src="https://github.com/brucejk/LiDARTag/blob/master/figure/figure1.png" width="640">

# Why LiDAR?
Robust to lighting!!  
<img src="https://github.com/brucejk/LiDARTag/blob/master/figure/lighting1.jpg" width="240">
<img src="https://github.com/brucejk/LiDARTag/blob/master/figure/lighting2.jpg" width="240">
<img src="https://github.com/brucejk/LiDARTag/blob/master/figure/lighting3.jpg" width="240">

## Overall pipeline
<img src="https://github.com/brucejk/LiDARTag/blob/master/figure/pipeline.png" width="960">


## Speed Analysis
<img src="https://github.com/UMich-BipedLab/LiDARTag/blob/release_v0/figure/LiDARTagAnalysis.png" width="960">


## Presentation and Video
Introduction Video
Please checkout the introduction [video](https://www.brucerobot.com/lidartag). It highlights some importants keypoints in the paper!  

<img src="https://github.com/UMich-BipedLab/LiDARTag/blob/release_v0/figure/LiDARTagIntro.png" width="960">

## Required packages
Please download [_LiDARTag_msgs_](https://github.com/UMich-BipedLab/LiDARTag_msgs) and place them under a catkin workspace.

## Installation
TODO

## Parameters
TODO

## Dataset
Please download from [here](https://drive.google.com/drive/folders/1MwA2dn6_U3rCWh9gxCe8OtWWgSxriVcW?usp=sharing).  

## Running
This package provides a launch file that you can directly run the package. 

## Citations
The detail is described in: 
Jiunn-Kai Huang, Maani Ghaffari, Ross Hartley, Lu Gan, Ryan M. Eustice,
and Jessy W. Grizzle, "LiDARTag: A Real-Time Fiducial Tag using
Point Clouds" ([PDF](https://github.com/UMich-BipedLab/LiDARTag/blob/release_v0/LiDARTag.pdf))([arXiv](https://arxiv.org/abs/1908.10349))

```
@article{huang2019lidartag,
  title={LiDARTag: A Real-Time Fiducial Tag using Point Clouds},
  author={Huang, Jiunn-Kai and Ghaffari, Maani and Hartley, Ross and Gan, Lu and Eustice, Ryan M and Grizzle, Jessy W},
  journal={arXiv preprint arXiv:1908.10349},
  year={2019}
}
```


