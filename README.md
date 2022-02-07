# SLAM Integrated Mobile Platform
University project to integrate a laserscanner, GNSS antenna and remote controled vehicle in order to perform SLAM and georeference the results. 

![GitCover](https://user-images.githubusercontent.com/92944341/152424731-54061cfb-b6c4-4ac2-ab5a-4ca5c56b4e54.png)


## Overview
This repository includes the processing code for data collected with the SIMP vehicle as well as some example data to run the program. The project is structured as follows:

```
 your_project_folder/
	│
	├── Code/  
        │   ├── functions/  
	│   ├── Georeferencing.m 
	└── Data/  
	    ├── GNSS Trajectories/  
	    ├── GoPro Images/  
            │   └── GoProSchloss/  
 	    ├── Scan Trajectories/  
	    ├── Scans/  
	    └── output/  
```

## How to use
The Georeferencing function includes the main code that will be used to call all functions related with this project. To process the example data simply run `Georeferencing.m` and select the corresponding data when prompted. 

The data can be downloaded from Google-Drive via this link (~ 2GB): <br />
https://drive.google.com/drive/folders/1lvRP46lA1Gb7gnMfUR8oFm5_vJhGkE-_?usp=sharing

Simply download "SIMP Data" and integrate the folders into `your_project_folder/Data/` to match the abovementioned project structure.

The following MATLAB Add-Ons need to be installed:
- Aerospace Toolbox (by MathWorks)
- Mapping Toolbox (by MathWorks)
- Signal Processing Toolbox (by MathWorks)
- Symbolic Math Toolbox (by MathWorks)
- Lidar Toolbox (by MathWorks)
- Image Processing Toolbox (by MathWorks)
- lasdata (by Teemu Kumpumäki)

After successfully running the code, the output pointcloud and trajectory can be found in `your_project_folder/Data/output/`

## Processing code summary
The main steps of the program are:
- Loading all necessary data (see Data folder)
- Calculating Time Offset (to connect Scan and GNSS data)
- Coarse trajectory match (as preparation for the ICP-based accurate matching)
- Accurate trajectory match (using temporally closest points of both trajectories)
- Estimate point matching accuracy (statistical values for relative accuracy of matching algorithm)
- Transform point cloud (apply combined transformation)
- Colorize point cloud (uses GoPro Data to determine colors)
- Remove moving objects (to clean up the point cloud)
- Ground classification (to seperate the points by ground and get some semantic information)
- Save final cloud (in Earth-Centered-Earth-Fixed (ECEF) coordinates)
