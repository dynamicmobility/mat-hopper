<p align="center">
  <h1 align="center">Materials Matter: Investigating Functional Advantages of Bio-Inspired Materials via Simulated Robotic Hopping
  </h1>
  <p align="center">
    <a href="https://hi.is.mpg.de/person/aschulz"><strong>Andrew K. Schulz*</strong></a>
    ·
    <a href="https://ayah-ahmad.github.io/"><strong>Ayah G. Ahmad*</strong></a>
    ·
	<a href="https://maegantucker.com/"><strong>Maegan Tucker*</strong></a>
    ·
</p>
<p>

  </p>
  <strong>*</strong> designates equivalent contributions to this GitHub repository. 
<h5 align="center">

[![Website shields.io](https://img.shields.io/website?url=http%3A//tokenhmr.is.tue.mpg.de)](https://dynamicmobility.github.io/ICRA2025-hopper-project-page/) 
[![YouTube Badge](https://img.shields.io/badge/YouTube-Watch-red?style=flat-square&logo=youtube)](https://www.youtube.com/watch?v=SEuEXRi80XU)
[![arXiv](https://img.shields.io/badge/arXiv-2409.09895-00ff00.svg)](https://arxiv.org/abs/2409.09895)
 
</h5>
  <p align="center"> 

<div style="display:flex;">
    <img src="assets/func-grad.gif" width="98%">
</div>


## Installation
This code was tested with:
- Ubuntu 22.4
- CUDA 11.8
- python 3.9 




# mat-hopper
This repository provides a simulation environment for one-legged hopping that models physical material designs. This repository is intended to accompany our ICRA 2025 submission titled ''Materials Matter: Investigating Functional Advantages of Bio-Inspired Materials via Simulated Robotic Hopping''.


## Simulation-Setup
Our framework utilizes the python bindings of MuJoCo (documentation: https://mujoco.readthedocs.io/en/stable/python.html)
To install this on your computer, you should only have to run the following
``` 
pip install mujoco
```
Note: DO NOT try and install `mujoco_py`, this is an outdated and no-longer-maintained version of mujoco python.

After installing `mujoco`, you can test your installation by opening the standalone app:
```
python -m mujoco.viewer
```

### Run Simulations of Mono-Materials
To run the first set of experiments, run the python script `run_experiment_one.py`:
```
python run_experiment_one.py
```

## Run Simulation Sweeps (across material density and material stiffness separately)
To run the second set of experiments, run the python script `run_experiment_two.py`:
```
python run_experiment_two.py
```

## Run Simulations of Functional Gradients
To run the third set of experiments, run the python script `run_experiment_three.py`:
```
python run_experiment_three.py
```

## Run Simulation Speed Experiments
To obtain the heatmap depicted in Figure 4, run the python script `run_experiment_simtimes.py`:
```
python run_experiment_simtimes.py
```

## Simulate Hopping
Lastly, to simulate a single instance of one-legged hopping, run the python script `test_hopping.py`
```
python test_hopping
```
Within this script, you can select which behavior you would like to run, and which xml you would like to utilize. 

# Citation

```bibtex
@misc{schulz_materials_2024,
	title = {Materials Matter: Investigating Functional Advantages of Bio-Inspired Materials via Simulated Robotic Hopping},
	author = {Schulz, Andrew K. and Ahmad, Ayah G. and Tucker, Maegan},
	year = {2024},
}

```
## License
See the LICENSE file for more information. 

## Acknowledgements
The authors would like to acknowledge that much of this work is possible with the help of several different repositories including [the Mujoco XML Reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html). 

The authors thank the Alexander von Humboldt Foundation and International Max Planck Research School for Intelligent Systems, [IMPRS-IS](https://imprs.is.mpg.de/) for supporting AKS. The authors thank the authors of the [BITE paper](https://bite.is.tue.mpg.de/) for this Readme template. Thanks to [Katherine J. Kuchenbecker](https://is.mpg.de/~kjk) for the support. 

## Contact 

This code repository was implemented by [Andrew Schulz](https://github.com/Aschulz94), [Ayah Ahmad](https://github.com/ayah-ahmad), and [Maegan Tucker](https://github.com/maegant) in collaboration between [The Dynamic Mobility Group](https://dynamicmobility.github.io/) at Georgia Tech and the [Haptic Intelligence Department](https://hi.is.mpg.de/) at the Max Planck Institute for Intelligent Systems - Stuttgart. 

Give a ⭐ if you like our work. 

