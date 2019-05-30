# UWB-IMU-Optical datasets for EKF-based sensor fusion experiments

The package comprises of multiple log files in .csv format and matlab scripts in .m format.

The log files were recorded from the experiments done in the ICRA2018 paper "Robust Target-relative Localization with Ultra-Wideband Ranging and Communication" by Thien-Minh Nguyen et. al.

The matlab scripts are written to calculate the accuracy and plot the estimates.

# Usage

Please download the package, extract and run the matlab scripts.

You may want to have 2 monitors with resolution 1920x1080 to have the figures nicely tiled up on the second monitor while running matlab on the first monitor like setup.

Each matlab script will plot the log files corresponding to the reported experiments in the paper, namely near anchor, far anchor and moving anchor experiments.

Note that at the beginning of each matlab file there is a line to declare the log file name, followed by a switch case block where the relevant log files are listed, take the file **anteplot_2anc_px4_sq_near.m** as an example:

...

% Choose one of the log file in the switch below
logname = 'niv20170813_sq_60_10';
% Set the setpoint file
setpoints = csvread('square_90_15.csv');
setpoints = setpoints';

switch logname
    
    % These experiments do not have correlation flow
    case 'niv20170813_sq_60_10'
        % square_60_10
        tstart = 55;
        tend = 266.5;
        
    case 'niv20170813_sq_90_15'
        % square_90_15
        tstart = 65;
        tend = 206.5;
...

You can change the logname variables with the relevant log file name in listed in the switch block.

# Reference

If you are interested, please cite the following work:

@inproceedings{nguyen2018robust,
  title={Robust Target-relative Localization with Ultra-Wideband Ranging and Communication},
  author={Nguyen, Thien-Minh and Zaini, Abdul Hanif and Wang, Chen and Guo, Kexin and Xie, Lihua},
  booktitle={2018 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={2312--2319},
  year={2018},
  organization={IEEE}
}

@inproceedings{nguyenultra,
  title={An Ultra-Wideband-based Multi-UAV Localization System in GPS-denied environments},
  author={Nguyen, Thien Minh and Zaini, Abdul Hanif and Guo, Kexin and Xie, Lihua},
  booktitle={2016 International Micro Air Vehicles Conference},
  year={2016}
}

# Thanks for your interest
