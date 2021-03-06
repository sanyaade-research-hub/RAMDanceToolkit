# RamDanceToolkit

This repository is the primary source for the "[Reactor for Awareness in Motion](http://www.ycam.jp/en/performingarts/2013/02/ram-workshop.html)" Joint Research Project with Yoko Ando at [Yamaguchi Center for Arts and Media](http://www.ycam.jp). The other source is the [Motioner](https://github.com/YCAMInterlab/Motioner) application for accessing data from the custom inertial motion sensor system.

The toolkit itself is built using a project file in `libs/`. This toolkit is used by applications in `apps/` such as `RAMDanceToolkit` which demonstrates a number of scenes built with the toolkit. `addons/` contains submodules that refer to addons which are not shipped with openFrameworks.

The only app that depends on an addon which is not in a submodule is `apps/OpenNIOSC`.  If you're interested in building this application, make sure you have a copy of [this version's develop branch](https://github.com/kylemcdonald/ofxOpenNI/tree/develop) in your addons folder.

`dev/` contains work in progress or unused but interesting code.

## OSC

The OSC specification for RAM is as follows:

* Applications generally receive on port `10000`.
* Skeleton data is sent as a series of nodes in a single OSC message at the address `/ram/skeleton`.

The structure of each OSC message is:

1. `s`: the actor name.
2. `i`: the number of nodes in the message.
3. Array of nodes.
4. `f`: the message timestamp.

The arguments for each node are:

1. `s`: name of the node.
2. `fff`: (x, y, z) position of the node.
3. `ffff`: angle-axis orientation of the node, stored as (angle, x, y, z).

## Tracking

Some apps related to tracking can be found in the `dev/` folder.

### CircleTracking

This app integrates data from multiple Kinects in order to track a single bright point in real time. We've tested this system with up to three Kinects, using libfreenect via ofxKinect, on a desktop Mac Pro. The best target for tracking is a retroreflective ball as used in motion capture, but a diffuse infrared LED will also work.

After opening the app, six streams will be displayed from the three Kinects, showing video and depth information. Pressing the "Calibrate" button under the "Background" heading will sample any bright points in the background and ignore them, in order to keep the tracking as robust as possible by avoiding distractions. After the background is calibrated, press "Calibrate" under the "Registration" heading. This will begin the spatial calibration procedure. Slowly wave the tracked point through the space. For every frame in which the tracked point is visible from the first camera and another camera, a note will be made about that relationship. As more relationships are stored, the app will attempt to find a configuration (position and orientation) of each of the sensors that explains the data best. To speed up calibration, you can increase the "Calibration rate" parameter, but this can also mean that the data is recorded from the different sensors at slightly different times, leading to noise in the calibration.

The calibration data is automatically stored in the `data/` folder using a filename that corresponds to the serial number of the Kinect. When the app is opened the next time, these calibration files are loaded automatically. However, the background must be re-calibrated.

If more than one point is visible, the system will not be able to reconstruct the positions. Further work must be done to adapt this system to multiple point tracking, as well as sending data over OSC. However, the primary difficulty is currently that the 3d data from the tracking system needs to be tracked more accurately. Right now the 3d positions are extracted by looking at the area around the tracked point, and averaging those locations. Another technique, such as fitting a plane and ignoring outliers in the averaging, may lead to more robust data.

The primary innovation of the CircleTracking app is the use of OpenCV's `estimateAffine3D()` function, which solves for the position and orientation of the sensors based on an original data set.