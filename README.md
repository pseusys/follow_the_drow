# Follow the DROW

This project studies DROW person detector and algorithmic detector performance.

It also contains a pipeline for using different data sources and detectors as ROS nodes.

Finally, it suggests another neural network architectures that might perform better than DROW detector neural network.

## Contents of the repository

This repository consists of several parts:

1. Library - a library of detectors, datasets and other utilities, written in python and c++ with interoperability means.
2. ROS system - a system for running data source, detectors and visualization pipeline in ROS nodes inside of a Docker container (using library).
3. Research part - Jupyter notebooks for visualization and testing library detectors properties and performance.

## Library

Library is located inside of the `library` directory.

### C++ library

C++ library is located inside of the `library/cpp_core` directory.

It is a C++ static library, it can be compiled and installed with special `make` command:

```bash
make build-lib
```

> NB! The command will install the library only if it was run by a superuser.

All the library classes and functions belong to `follow_the_drow` namespace.

The library includes:

1. `AlgorithmicDetector` class.
   This class implements an algorithmic person detector.
   Detector arguments are provided to it as constructor arguments.
   The detectrion can be done by using `forward` method with different arguments number.
   This method accepts detections as `Point`s and returns vector of `Point`s.
2. `Cluster` class.
   This class represents a cluster of points, situated next to each other.
   Its' constructor accepts first and last point **indexes** and the whole detected points vector.
   It has several useful methods for calculating different cluster properties:
   - `size`: distance from start to end points.
   - `start`: start point index.
   - `middle`: middle point index.
   - `end`: end point index.
   - `startPoint`: start point.
   - `middlePoint`: middle point.
   - `betweenPoint`: the point between cluster start and end.
   - `endPoint`: end point.
   - `distanceTo`: distance to other class.
   - `middleBetween`: the point between between points of this cluster and other cluster.
   > NB! If a cluster consists of 3 points: (0, 1), (0, 0) and (1, 0) - the middle point will be (0, 0) and the between point will be (0.5, 0.5).
3. `Point` class.
   This class represents a geometrical 2D point.
   It has several useful methods for working with points:
   - `polarToCartesian`: converts polar data (distance and angle) to point.
   - `distanceTo`: distance between two points.
   - `middleBetween`: point between two points.
   - ...also operators for points addition and substraction are defined.
4. `Tracked` class.
   This class represents a tracked point: point, including frequency and uncertainty.
5. `utils` file contains utility functions.
6. `binding` file includes classes and functions for Python interoperability.
   `PythonDetectorFactory` provides a pythonic interface for using `AlgorithmicDetector` class.
   > NB! This file won't be included into standalone library distribution!

### Python library

Python library is located inside of the `library/follow_the_drow` directory.

## ROS ecosystem

## ROS package

This project can be run not only locally, but also on RobAIR.
However, a few things have to be doublechecked for that:

1. Connection (host IP and RobAIR IP): see `Makefile`.
2. **Input** topics and frames names: see `robair_ufr.launch` in `robairmain` package on RobAIR and `deploy/conf.env`.
3. Follow me behavior nodes names and packages: see `~/catkin_ws/src` on RobAIR and `deploy/conf.env`.

> _WARNING!_ Make sure ROS is **not** running on RobAIR - that would lead to node name collisions!

Run it with this command:

```bash
make launch-docker-robot
```

> Nodes are _by default_ built with debug info **and** GDB is included into Docker image.

In order to run any of the nodes with GDB, add `launch-prefix="gdb -batch -ex run -ex bt --args"` to node arguments (it is added by default).

In file `deploy/follow_the_drow/CMakeLists.txt` in line #5 `RelWithDebInfo` can be changed to `Release` for release artifact producing or to `Debug` for debug artifact producing.

### Debug

## Notebooks

## Other

## Notes

### DROW dataset

Contains many sequences, each sequence has:

1. `.bag.csv` file - 1st column contains unique index, 2nd column contains timestamp, other columns contains scans data.
2. `.bag.odom2` file - 1st column contains unique index, 2nd column contains timestamp, three last columns contain odometry data (X-axis, Y-axis and angle).
3. `.bag.wa` file - 1st column contains unique index, second - array of **walker** detections.
4. `.bag.wc` file - 1st column contains unique index, second - array of **wheelchair** detections.
5. `.bag.wp` file - 1st column contains unique index, second - array of **person** detections.

### RobAIR FoV

RobAIR lidar has 726 rays, it covers ??? degrees.
RobAIR camera has more narrow field of view: it starts on ray **315** and ends on ray **430**.

### DROW detector frequency

In the DROW paper system frequency was 12.5 Hz.
Here, it was decided to lower this number to 10 Hz for compatibility reasons.

> NB! Neither the laptop the system was launched, nor RobAIR have GPU, so neural network based DROW detector operates _really_ slowly (at approximately 1 Hz).

## TODOs

1. In the DROW paper translation odometry data was not used while calculating "cutouts" - using it might improve performance.
2. The DROW dataset is very poorly annotated - it can be re-annotated (by and or algoritmically) in order to improve real world performance.
3. The DROW detector neural network uses complicated "cutouts" system in order to receive temporal data - it can be simplified (and probably improved) by using recurrent neural network.
4. The DROW detector neural network uses complicated "votes" system in order to calculate human positions - this can be simplified (and probably improved) by using a different neural network architecture:

Let `n` be the ray count.
Then neural network input shape will be `n` floats - `n` distances for `n` laser measures (or `2n` for 2 lidars).
The output shape will be `n` floats - each of them will be a **probability of the laser measurement to hit a person**.
Then the output data will be converted to coordinates using laser data.

## Roadmap

1. Setup follow me behavior interop (topics, connections)
2. Tracking node: consistent output
3. Complete readme(s)
4. Write a report
5. Fill in README gaps (`???`)
