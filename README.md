# Inverse-Optimal Safety Control for Hybrid Systems 

Simulation for examples in HSCC'25 paper: Inverse-Optimal Safety Control for Hybrid Systems 

Author: Carlos A. Montenegro G. and Santiago Jimenez
Revision: 0.0.0.3 Date: 01/29/2025
https://github.com/camonten/InverseOptimal_HybridGames

----------------------------------------------------------------------------
# `Structure of the project`

The project is structured as follows:

```
.
├── BouncingBall.m
├── ControlBarrierFunction.m
├── PhasePortraits.m
├── CostEvaluation.m
├── README.md
└── Figures.py
```

Files description:

- `BouncingBall.m`: HybridSystem subclass defining the flow set, flow map, jump set, and jump map of the one-degree-of-freedom juggling system.
- `ControlBarrierFunction.m`: Class defining ISS-CBFs for Hybrid Systems
- `PhasePortraits.m`: Code necessary to reproduce Fig. 1. in the paper.
- `CostEvaluation.m`: Code necessary to reproduce Fig. 2. in the paper.


# `Installation`

Prerequisites:
- `Matlab` (Developed in R2022b). Install HyEQ Toolbox 3.0.0.76, available [at this link](https://www.mathworks.com/matlabcentral/fileexchange/41372-hybrid-equations-toolbox).

`IMPORTANT`: The code will not run if the HyEQ Toolbox is not previously installed.

# `Running the files`

Having successfully installed the HyEQ Toolbox:
1. Clone the repository and `cd` into it
```bash
git clone https://github.com/camonten/InverseOptimal_HybridGames.git
```
```bash
cd InverseOptimal_HybridGame
```
2. Open Matlab and make sure your `Current Folder` is this repository you just cloned.
3. To reproduce `Fig. 1` in the paper you should run `PhasePortraits.m`. The result should be the following:
![Fig. 1](Figures/PhasePortraits.png =250x)
4. To reproduce `Fig. 2` in the paper you should run `CostEvaluation.m`. The result should be the following:
![Fig. 2](Figures/CostEvaluation.png =250x)