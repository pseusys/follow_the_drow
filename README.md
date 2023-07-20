# Follow the DROW

## Debug

In order to run any of the nodes with GDB, add `launch-prefix="gdb -ex run --args"` to node arguments.

> [`GDB`](https://www.sourceware.org/gdb/) is already bundled into Docker image!

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

1. Test "cutouts" with translation data
