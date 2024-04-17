# Crazyswarm2 Tools

This repository is based on the branch ```tune-lee``` from [Crazyswarm2](https://github.com/IMRCLab/crazyswarm2/tree/tune-lee). It contains all the data that was accumulated during the tuning experiments of the Lee controller with the standard and upgraded Crazyflie hardware. The data is stored in the folder ```/logs```. The information (and controller gains) for each experiment is stored in the folder ```/info```. In here, you will also find the script ```plot.py``` for yaml-based automatic report generation to process the logs and info files. The script ```data_helper.py``` focuses on manipulating and adding data to the automatically generated reports. In the following, you will find a guide on how to use the scripts and data. Also, there is the script `model_payload.py` which is used for calculating the residual forces and torques with the UAV payload model from [this paper](https://mathweb.ucsd.edu/~mleok/pdf/LeLeMc2010_quadrotor.pdf). To find some information on UAV dynamics, please refer to [this paper](https://whoenig.github.io/publications/2021_T-RO_Shi.pdf).

> All flight experiments conducted for the current log and info files used the motion capture system OptiTrack.

# General Information

- The data of each experiment is stored in the folder ```logs```

> Note: A log file may have the name ```log14```, or if it is a repetition ```log14_2```. This is called the *logging number*. Please format your logs accordingly.

- The parameters and information of each experiment is stored in the folder ```info```

> Note: The correponding info file would have the name ```info14.yaml```. Please note that there is no suffix because one configuration of parameters describes all duplicates of an experiment. It is called the *experiment number*. You have to enter the experiment number in the file ```crazyflie/config/crazyflies.yaml``` later on so that it will be saved. See the next sections for more information.

- The finished reports combine the data and experiment information and are stored in the folder ```reports```

> Note: The reports are not uploaded to this repository because they are too large. You can create them automatically in one run by yourself, see the next section for more information.

# FAQ

- [Crazyswarm2 Tools](#crazyswarm2-tools)
- [General Information](#general-information)
- [FAQ](#faq)
  - [Where to look for information on the tuning experiments? Where to find the files to repeat the flight experiments?](#where-to-look-for-information-on-the-tuning-experiments-where-to-find-the-files-to-repeat-the-flight-experiments)
  - [Which experiments where conducted with which type of hardware?](#which-experiments-where-conducted-with-which-type-of-hardware)
  - [Which gains for the Lee Controller work well with the upgraded hardware?](#which-gains-for-the-lee-controller-work-well-with-the-upgraded-hardware)
  - [Which gains for the Lee Controller work well with the standard hardware?](#which-gains-for-the-lee-controller-work-well-with-the-standard-hardware)
  - [I want to see the reports of previous flight experiments. Where can I find them?](#i-want-to-see-the-reports-of-previous-flight-experiments-where-can-i-find-them)
  - [I would like to plot the data of an experiment. How do I generate a report?](#i-would-like-to-plot-the-data-of-an-experiment-how-do-i-generate-a-report)
  - [How to conduct a flight experiment using on the workstation?](#how-to-conduct-a-flight-experiment-using-on-the-workstation)
  - [How to change the settings for a report?](#how-to-change-the-settings-for-a-report)
  - [Where can I find the controller gains for the Crazyflie with standard hardware when using the Lighthouse motion capture system?](#where-can-i-find-the-controller-gains-for-the-crazyflie-with-standard-hardware-when-using-the-lighthouse-motion-capture-system)

## Where to look for information on the tuning experiments? Where to find the files to repeat the flight experiments?

As this repository is based on the branch `tune-lee` from [Crazyswarm2](https://github.com/IMRCLab/crazyswarm2/tree/tune-lee), it contains all logs and info files required for generating reports. The controller gains of each experiments as well as the used hardware can be found in the respective info files. To repeat the flight experiments you may use the branch `tune-lee` from [Crazyswarm2](https://github.com/IMRCLab/crazyswarm2/tree/tune-lee) because it offers the following scripts.

1. `crazyflie/config/crazyflies.yaml`: This script contains the configuration for each flight experiment. The last flight experiment conducted was the one with the payload uav and the motion capture system. The payload had a weight of 5 grams.  

2. `crazyflie_examples/crazyflie_examples/lee.py`: This script contains the automatic loading process of the trajectory from the configuration file `crazyflies.yaml`. It starts logging before starting the trajectory, and ends it right after the end of the flown trajectory. Please connect the Crazyradio to the URI `radio://0/80/2M/E7E7E7E70B` which corresponds to Crazyflie 11.

> More information on conducting the experiments can be found below.

## Which experiments where conducted with which type of hardware?

The logs and reports up until number `145` were done with the upgraded CF motors and QProp propellers. The logs and reports from number `146` and onwards were done with the standard motors and propellers. For more information on the used hardware, flown trajectory, and controller gains please refer to the info files.

## Which gains for the Lee Controller work well with the upgraded hardware?

The controller gains from the info file `145` work well with the upgraded hardware.

## Which gains for the Lee Controller work well with the standard hardware?

The controller gains from the info files `178` and `182` work well with the standard hardware.

## I want to see the reports of previous flight experiments. Where can I find them?

Most of the reports can be found on the workstation in the flightspace. Please log in as `Dennis Schmidt`. The password could be provided by either Khaled or Wolfgang as it is located in our group chat. The path to look for the reports is `~/projects/ros2_ws/src/crazyswarm2/_experiments/reports`.

## I would like to plot the data of an experiment. How do I generate a report?

0. Create a virtual environment and install the requirements.
1. Check if you have the folder ```/reports```. If not, create it.
2. Execute the script ```plot.py```. Currently, the script is in the mode ```manual single``` which is why it will ask you which log you want to plot. The newest log and info files are `201` up to `206`. These logs were created to calculate the residual forces and torques of UAV with a payload of 5 grams. Type the number of the desired log you want to process into the terminal when prompted. Note that there are different modes available. You can change the mode in the script ```plot.py```.

    ```bash
    cd crazyswarm2-tools
    python plot.py
    ```

    It should output something similar to this:

    ```bash
    ...creating figures
    start_time: 143230.548
    output path: reports/log29.pdf
    processing event: fixedFrequency (0)
    >>> created figure 1: Positions
    >>> created figure 2: Translation Velocities
    >>> created figure 3: Angles
    >>> created figure 4: Angular Velocities
    >>> created figure 5: Thrust
    >>> created figure 6: Torques (xyz)
    >>> created figure 7: Trajectories
    ...done creating figures
    ```

3. Check the folder ```/reports``` for the report. It should have the name `log206.pdf`.
4. If you wonder how to add additional data that is not included in the logs but computed on the basis of it, for example by fitting a cubic spline, check out the function ```add_data()``` in ```plot.py``` as well as the script ```data_helper.py```. The file ```settings.yaml``` defines the requirements for the computation of additional data. There, multiple methods are defined to generate and add new data to the report from the logs. For example, to add the residual forces to the report `206` the strings `residual.fx`, `residual.fy`, and `residual.fz` were added to the section `additional_data` of the file `settings.yaml`. The calculation of the residual forces and the payload uav model can be found in the file ```model_payload.py```. There, you can also find code to fit data using cubic splines (with smoothing) and calculate the residual torques, as well as the state errors of the payload uav model.

> Note that to this date, the log folder contains logs up to the number `206`. This repository developed gradually and new field names were introduced to the `settings.yaml` over time. Therefore, it is not possible to plot old logs, e.g. `log47`, with the newest version of the `settings.yaml` out of the box. In order to plot these logs you would need to change the `settings.yaml` accordingly. An easy way to grab the compatible settings for report generation is to check the date of the respective log in the commit history and go back in time with a `git checkout` to that commit on the branch `main`. I can recommend the VS Code extension `Git Graph` for browsing the commit history.

## How to conduct a flight experiment using on the workstation?

0. If not already there, create the three folders mentioned above in the main directory of this repository: ```logs```, ```info```, ```reports```

1. Decide which experiment you want to do
    - Decide which timescale, trajectory, motors, propellers and parameters of the lee controller, and experiment number you want to use: ```crazyflie/config/crazyflies.yaml``` (field ```ctrlLeeInfo``` and ```ctrlLee```)
    - There are 3 different motors in the lab at the current moment:
        1. Standard motors (CF)
        2. Upgraded motors (CF)
        3. Upgraded motors (BETAFPV)
    - There are 2 different propellers in the lab at the current moment:
        1. Standard propellers (CF)
        2. Upgraded propellers (QProp)

2. Check that the sd card includes this config file which is specific to the plotting script later on: ```config.txt```. You may have to adjust the file depending on your needs.

> Note that you can not use the `config.txt` directly on the sd card. Please create a new file with the same name and copy either the content for the tuning or modeling experiments.

3. Save your tuning parameters and experiment information in the folder ```/info``` by executing the following command in the terminal

```bash
cd _experiments
python save.py
```

> This script generates the info file for the experiment, i.e. writes the current parameters of the file ```crazyflie.yaml``` into a new info file. The info file can be used with the corresponding log file to generate a report. Please adjust the filepath of the file ```crazyflie.yaml``` in the script ```save.py``` to your needs. It is recommended to save the parameters before doing an experiment so that you can easily reproduce the experiment later on.

4. Run the experiment, the script ```crazyflie_examples/crazyflie_examples/lee.py``` will automatically read the parameters from the file ```crazyflie/config/crazyflies.yaml```

5. Rename and transfer the logs from the sd card to the workstation into the folder ```/logs```. Be sure to name the logs correctly, i.e. `log213`.

## How to change the settings for a report?

In the following you will find some (older) information on changing the parameters in the `settings.yaml`. You can also find more information in the file itself, epsecially in the section `additional_data`. 

- Be sure to have the folders ```/reports``` and ```/logs``` in this repository
- Adjust the following general settings
    - ```event_name```
    - ```output_dir```
    - ...
- Figures can be defined in the figure settings
    - Copy and change the following parameters
        - ```title```
        - ```type``` (only 2d and 3d for now)
        - ```x_label```
        - ...
        - Each structure within a figure dictionary defines a subplot
            - The x-axis data of the subplot is defined by the key, i.e. ```timestamp``` which is the name of the time data wihtin the log file
            - Each subplot can have multiple signals which can be defined by another dictionary with the key ```y_info```
                - It has the keys ```data``` and ```label```
                - ```data``` defines the name of the signalss within the log file by its keys, the legend names of the signals can be defined with the respective values
- Title settings can also be adjusted
    - ```title_settings```: change the names of the settings which should be displayed in the title
    - ```title_params```: define the parameters which should be displayed in the title
- Change the unit settings
    - ```convert_units```: define the units which should be converted and how, the value represents the conversion factor
    - ```units```: define the string which should be displayed as the unit (not in use yet)
  
## Where can I find the controller gains for the Crazyflie with standard hardware when using the Lighthouse motion capture system?

The current controller gains can be found in the file `/home/denni/projects/crazyswarm2-tools/cflibsetparams.py`. This file was used to load the Lee controller and its gains onto the Crazyflie. The GUI of the `cfclient` can be used for flying witht the Crazyflie afterward. The controller gains deviate from the controller gains used for the motion capture system because the Crazyflie used to make a backflip on landing. Older variations of the controller gains can be found in the commit history of the branch `residual-forces`.

> Please connect the Cherry keyboard to the workstation for now because the other keyboard will be interpreted as a game controller by the `cfclient`.
