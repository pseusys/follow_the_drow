# Follow the DROW

## Debug

> Nodes are _by default_ built with debug info **and** GDB is included into image.

In order to run any of the nodes with GDB, add `launch-prefix="gdb -batch -ex run -ex bt --args"` to node arguments.
In file `deploy/follow_the_drow/CMakeLists.txt` line #5 can be changed to `Release` for release artifact or to `Debug` for debug artifact.

## RobAIR interop

This project can be run not only locally, but also on RobAIR.
However, a few things have to be doublechecked for that:

1. Connection (host IP and RobAIR IP): see `Makefile`.
2. **Input** topics and frames names: see `robair_ufr.launch` in `robairmain` package on RobAIR and `deploy/conf.env`.
3. Follow me behavior nodes names and packages: see `~/catkin_ws/src` on RobAIR and `deploy/conf.env`.

Run it with this command:

```bash
make launch-docker-robot
```

> _WARNING!_ Make sure ROS is **not** running on RobAIR - that would lead to node name collisions!

## REPORT

### DROW dataset

Contains many sequences, each sequence has:

1. `.bag.csv` file - 1st column contains unique index, 2nd column contains timestamp, other columns contains scans data.
2. `.bag.odom2` file - 1st column contains unique index, 2nd column contains timestamp, three last columns contain odometry data (rotation? translation?)
3. `.bag.wa` file - 1st column contains unique index, second - array of **walker** detections.
4. `.bag.wc` file - 1st column contains unique index, second - array of **wheelchair** detections.
5. `.bag.wp` file - 1st column contains unique index, second - array of **person** detections.

### RobAIR FoV

- Start of FoV: 315
- End of FoV: 430
- Number of rays: 726

### DROW detector frequency

It's 12.5 in paper, but here I decided to use 10 for compatibility and annotations

## TODOs

1. Test "cutouts" with translation data?

## Roadmap

1. Debug and run stateless detector
2. Rewrite stateful detector, make sure it inherits from abstract
3. Debug and run it in wrapper
4. Setup follow me behavior interop (topics, connections)
5. Detector interface (including pythonic) should accept odometry
6. Fix TODOs
7. Complete workflows
8. Follow me: Study inputs, is it first detected human or closest? Create adapter node, input humans, output one human, with modes; first_index, closest, first_detected (with tracking).
9. Move colors to visualizer
10. Move defines to parameters
11. Clear docker image on make clear
