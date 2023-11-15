# Crazyswarm2 Tools

This repository is based on the branch ```tune-lee``` from [Crazyswarm2](https://github.com/IMRCLab/crazyswarm2/tree/tune-lee). It contains all the data that was accumulated during the tuning experiments of the Lee controller. The data is stored in the folder ```_logs```. In here, you will a script for yaml-based automatic report generation to process the logs. The sciprt ````model.py`` focuses on manipulating and adding data to the automatically generated reports. In the following, you will find a guide on how to use the scripts and data.

# I just want to plot the data of an experiment. What should I do?

1. Check if you have the folder ```_experiments/reports```. If not, create it.
2. Execute the script ```plot.py``` in ```_experiments```. Currently, the script is in the mode ```manual single``` which is why it will ask you which log you want to plot. The current one I am using is ```182```. 
3. Wait for all plots to be generated. This may take a while.
4. Check the folder ```_experiments/reports``` for the report. It should have the name ```log182.pdf```.
5. If you wonder how to add additional data that is not included in the logs but computed on the basis of it, for example by fitting a cubic spline, check out the function ```add_data()``` in ```plot.py``` as well as the script ```model.py```. The file ```settings.yaml``` defines the requirements for the computation of additional data. There are two methods currently, using a single polynomial or cubic splines.

# How can I reuse the data and code from the tuning experiments?

## 1. data

- The data of each experiment is stored in the folder ```logs```

> Note: A log file may have the name ```log14```, or if it is a repetition ```log14_2```. This is called the *logging number*. Please format your logs accordingly.

- The parameters and information of each experiment is stored in the folder ```info```

> Note: The correponding info file would have the name ```info14.yaml```. Please note that there is no suffix because one configuration of parameters describes all duplicates of an experiment. It is called the *experiment number*. You have to enter the experiment number in the file ```crazyflie/config/crazyflies.yaml``` later on so that it will be saved. See the next sections for more information.

- The finished reports combine the data and experiment information and are stored in the folder ```reports```

> Note: The reports are not uploaded to this repository because they are too large. You can create them automatically in one run by yourself, see the next section for more information.

## 2. code

0. If not already there, create the three folders mentioned above in the directory ```_experiments```

1. Decide which experiment you want to do
    - Decide which timescale, trajectory, motors, propellers and parameters of the lee controller, and experiment number you want to use: ```crazyflie/config/crazyflies.yaml``` (field ```ctrlLeeInfo``` and ```ctrlLee```)
    - There are 3 different motors in the lab at the current moment:
        1. Standard motors (CF)
        2. Upgraded motors (CF)
        3. Upgraded motors (BETAFPV)
    - There are 2 different propellers in the lab at the current moment:
        1. Standard propellers (CF)
        2. Upgraded propellers (QProp)

> Note: The logs and reports up until number 145 were done with the upgraded CF motors (2) and QProp propellers (2). The logs and reports from number 146 onwards were done with the standard motors (1) and propellers (1).

2. Check that the sd card includes this config file which is specific to the plotting script later on: ```_experiments/config.txt```

3. Save your tuning parameters and experiment information in the folder ```_experiments/info``` by executing the following command in the terminal

```bash
cd _experiments
python save.py
```

4. Run the experiment, the script ```crazyflie_examples/crazyflie_examples/lee.py``` will automatically read the parameters from the file ```crazyflie/config/crazyflies.yaml```

5. Rename and transfer the logs from the sd card to this workstation into the folder ```_experiments/logs```. Be sure to name the logs correctly, see the first section for more information

6. Create the report, more information can be found in the next section

# How to create reports with custom plots?

The report is generated by the Python script ```plot.py``` which uses the settings defined in ```settings.yaml```, the log files and the experiment information stored in ```logs``` as well as ```info```. The info file has to have the same experiment number at the end of the string as the corresponding log file. The report is stored in the folder ```reports```.

## 1. settings.yaml

- Be sure to have the folders ```/reports``` and ```/logs``` in the directory ```_experiments```
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

## 2. plot.py

```bash
cd _experiments
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