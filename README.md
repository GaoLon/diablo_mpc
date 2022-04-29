# diablo_mpc

## Quick Start

A **MPC** for **diablo** configuration(see as differential car).
If you want to use the code, you should install `OSQP`, `osqp-eigen` and`serial` first, and can following steps:

```
cd test_mpc
catkin_make
source devel/setup.bash
roslaunch mpc test_mpc.launch
```

## Details

See in [markdown_file](./README.markdown)

### Notation

* Models are just for forward tracking, but we can track backward by inversing the model. Besides, the algorithm will choose a better way to track the path.
* When prediction, we don't choose just one point to linearize the model, that isn't precise enough.
* MPC will run iteratively until converge or at max loops.
* 100Hz with 100 predict steps is recommanded for consideration of effect and efficiency.
