# GUILDA Python

# First README.md edit - 2023.02.16

This document is under construction, we apologize for any inconveniences that it might cause.

## Index

- [Description](#anchor1)
- [Functionality](#anchor2)
- [Documentation](#anchor3)
- [Requirements](#anchor4)
- [Installation](#anchor5)
- [Source Code Explanation](#anchor6)
- [Cite](#anchor7)
- [Lincense](#anchor8)
- [Acknowledgement](#anchor9)
- [Contribute](#anchor10)

***

<a id="anchor1"></a>

## Description

GUILDA Python is the Python version of the original MATLAB based program [GUILDA](https://github.com/guilda-dev/guilda). While both programs are ment to be equivalent versions of each other, GUILDA is developed in MATLAB and documented in Japanese, while GUILDA Python is developed in Python and documented in English.

GUILDA stands for Grid & Utility Infrastructure Linkage Dynamics Analyzer.
GUILDA is a numerical simulator platform for smart energy management. The purpose of this program is to provide students and researchers in the field of systems and control engineering with an advanced numerical simulation environment that can be used with minimal knowledge of power systems. To achieve this, it is recommended to use this program in closeness with the textbook ["Power Systems Control Engineering: Systems Theory and MATLAB Simulation"](https://www.coronasha.co.jp/np/isbn/9784339033847/) (only available in its original language, Japanese. The English translation is currently being developed). As in this textbook the authors explain the structure and mathematical fundamentals of power systems in the language of systems and control engineering.

We are devising a way for students to learn the mathematical fundamentals and the construction of a numerical simulation environment in parallel. Through these activities, we aim to establish power systems as one of the familiar benchmark models in the field of systems and control, thereby helping to promote power system reform through the technologies and knowledge in this field.

This is an in-development project by：
- [Ishizaki Laboratory](https://www.ishizaki-lab.jp/home) at [Tokyo Institute of Technology](https://www.titech.ac.jp/english#)
- [Assistant Professor Takahiro Kawaguchi](http://hashi-lab.ei.st.gunma-u.ac.jp/~hashimotos/member/kawaguchi/en/index.html) of Gunma University.

To learn more visit [here](https://www.ishizaki-lab.jp/guilda).

***

<a id="anchor2"></a>

## Functionality

As a numerical simulator platform, GUILDA has the following main capabilities:

- Define power system network models, composed of generators, loads, buses, and lines. The networks can be based on standard models (e.g., IEEE-68), as well as personilized models.
- Implement controllers (local or global) to such network models.
- Simulate linear and non-linear models (with or without controllers / faults).

To achieve the above, the following functionalities are present:

- Linearize models and express them in state space representation.
- Calculate power flow.
- Determine equilibrium point (if not defined).

***

<a id="anchor3"></a>

## Documentation

Please visit the [official documentation site](https://guilda-dev.github.io/guilda-doc/). In it you will find several tutorials on:

- How to [define a power system model.](https://guilda-dev.github.io/guilda-doc/Reference/defineNet/0TopPage/)
- How to [add controllers.](https://guilda-dev.github.io/guilda-doc/Reference/addController/0TopPage/)
- How to [perform numerical simulations.](https://guilda-dev.github.io/guilda-doc/Reference/Analysis/net_simulate/)
- How to [derive an approximate linearized model of the created power system network model.](https://guilda-dev.github.io/guilda-doc/Reference/Analysis/net_getsys/)

Additionally, a demonstrative example of implementing, simulating, and adding control to a 3-bus network system is explained [here](https://guilda-dev.github.io/guilda-doc/SeriesAnalysis/0TopPage/).

***

<a id="anchor4"></a>

## Requirements

- Python version >=3.9
- C Compiler
- Fortran Compiler
- Sundials version >=5.1.0

- It will be used in simulation. See [this page](https://scikits-odes.readthedocs.io/en/stable/installation.html).
- Alternatively, installing from conda: `conda install -c conda-forge scikits.odes` will ease the process.

The main libraries are:

- `numpy`
- `scipy`
- `control`
- `assimulo`

Python Libraries (should we indicate the purpose of each library?):

- contourpy version 1.0.6
- control version0.9.3.post2
- cycler version 0.11.0
- fonttools version 4.38.0
- kiwisolver version 1.4.4
- matplotlib version 3.6.2
- numpy version 1.24.1
- packaging version 23.0
- pandas version 1.5.2
- pillow version 9.4.0
- pyparsing version 3.0.9
- python-dateutil version 2.8.2
- pytz version 2022.7
- scipy version 1.10.0
- scikits.odes version 2.7.0
- six version 1.16.0

***

<a id="anchor5"></a>

## Installation

1. Make sure to comply with all the requirements.
2. Download the source code from this site.
3. ...
4. Ready to use.

***

<a id="anchor6"></a>

## Source Code Explanation

In this section a brief description of the general structure of the source code is provided.

As a general rule, all classes and functions that are supposed to be called and used by the user directly, should start with a brief comment on its intended use and variable description. Thus, if there is a class or function that you want to use, please open its source code and look for such description.

### Classes

The classes that play a central role in GUILDA Python can be broadly divided into five types:

- [Power network.](https://guilda-dev.github.io/guilda-doc/SourceCode/power_network/)
- [Branch.](https://guilda-dev.github.io/guilda-doc/SourceCode/branch/)
- [Busbar.](https://guilda-dev.github.io/guilda-doc/SourceCode/bus/)
- [Equipment.](https://guilda-dev.github.io/guilda-doc/SourceCode/component/)
- [Controller.](https://guilda-dev.github.io/guilda-doc/SourceCode/controller/)

These five classes are the main classes from which the rest of classes are derived.

Here, the variables and methods implemented in these five main classes are explained. Other classes are not explained; however, as mentioned before, you can always check the description of the source code of your class of interest.

### Functions in folder "+function"

This directory stores functions that may be useful for improving work efficiency when using GUILDA Python.

### Tools in folder "+tools"

This directory is implemented by GUILDA Python. It stores functions used in the code of a class or function. It is created for the purpose of modularizing frequently used execution contents and improving the listing of source code. Therefore, the code under this directory is not a function that is used directly by the user, so we will omit the explanation.

***

<a id="anchor7"></a>

## Cite

To cite GUILDA Python please use the following:

/Insert how to cite/

***

<a id="anchor8"></a>

## License

GUILDA is licensed under the open source [MIT
License](https://github.com/guilda-dev/guilda/blob/333e544510994a0803767590c7adad1633d9d657/LICENSE).

***

<a id="anchor9"></a>

## Acknowledgement

The autors thank the continuous support of
/Insert list of supporters/

***

<a id="anchor10"></a>

## Contribute
