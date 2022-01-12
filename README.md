# Serial and Parallel Robot Kinematics
## System Requirements

Matlab R2021b:
- Curve Fitting Toolbox (version 3.6)
- Symbolic Math Toolbox (version 9.0)

Python 3.8 (included in `requirements.txt`):
```
cycler==0.11.0
fonttools==4.28.5
kiwisolver==1.3.2
matplotlib==3.5.1
numpy==1.22.0
packaging==21.3
Pillow==9.0.0
pyparsing==3.0.6
python-dateutil==2.8.2
six==1.16.0
torch==1.10.1
typing_extensions==4.0.1
```


## Scripts
### Part 1: Serial Robot
Matlab scripts for plotting:
- `workspace.m`
- `plot_task.m`
- `free_motion.m`
- `cartesian_motion.m`
- `avoidance_trajectory.m`

### Part 2: Parallel Robot
Matlab scripts for plotting:
- `final_course_work_parallel.m`

### Part 3: Gradient Descent for Inverse Kinematics
Visualisation of optimisation:

```
usage: lynxmotion_grad.py [-h] [--display] [--animate] [--verbose]

optional arguments:
  -h, --help  show this help message and exit
  --display   show strat/end plots of IK optimisation
  --animate   show animated plot of IK optimisation
  --verbose   verbose output during IK optimisation
```

Python scripts for testing and evaluation:
- `test_learning_rate.py`
- `test_tolerance.py`
- `test_restarts.py`
- `eval_tolerance.py`
- `eval_learning_rate.py`
- `eval_restarts.py`
