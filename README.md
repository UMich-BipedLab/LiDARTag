# LiDARTag
## Overview

This is a ROS package for detecting the LiDARTag via ROS.

* Author: Bruce JK Huang
* Maintainer: [Bruce JK Huang](https://www.brucerobot.com/), brucejkh[at]gmail.com
* Affiliation: [The Biped Lab](https://www.biped.solutions/), the University of Michigan

This package has been tested under [ROS] Kinetic and Ubuntu 16.04.
More detailed introduction will be updated shortly. Sorry for the inconvenient!

## Abstract
Image-based fiducial markers are widely used in robotics and computer vision problems such as object tracking in cluttered or textureless environments, camera (and multi-sensor) calibration tasks, or vision-based simultaneous localization and mapping (SLAM). The state-of-the-art fiducial marker detection algorithms rely on consistency of the ambient lighting. This paper introduces LiDARTag, a novel fiducial tag design and detection algorithm suitable for light detection and ranging LiDAR point clouds. The proposed tag runs in real-time and can process data faster than the currently available LiDAR sensors frequencies. Due to the nature of the LiDAR's sensor, rapidly changing ambient lighting will not affect detection of a LiDARTag; hence, the proposed fiducial marker can operate in a completely dark environment. In addition, the LiDARTag nicely complements available visual fiducial markers as the tag design is compatible with available techniques, such as AprilTags, allowing for efficient multi-sensor fusion and calibration tasks. The experimental results, verified by a motion capture system, confirm the proposed technique can reliably provide a tag's pose and its unique ID code. All implementations are done in C++ and available at: https://github.com/brucejk/LiDARTag

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


## Presentation and Video
https://www.brucerobot.com/lidartag

## Installation
TODO

## Parameters
TODO

## Examples
TODO

## Citations
The LiDARTag is described in: 

*Jiunn-Kai Huang, Maani Ghaffari, Ross Hartley, Lu Gan, Ryan M. Eustice and Jessy W. Grizzle “LiDARTag: A Real-Time Fiducial Tag using Point Clouds” (under review)


