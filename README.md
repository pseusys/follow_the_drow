# Follow the DROW

## Debug

In order to run any of the nodes with GDB, add `launch-prefix="gdb -ex run --args"` to node arguments.

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

1. Create wrapper node for detectors
2. Debug and run stateless detector
3. Rewrite stateful detector, make sure it inherits from abstract
4. Debug and run it in wrapper
5. Setup follow me behavior interop (topics, connections)
6. Rename data_outrigger.
