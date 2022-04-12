# Snow_Plower_4805
Autonomous Snow Plow Project for L1 Group 1 of SYSC 4805
# Team Members
- Alfred Akinkoye (101109483)
- Michael Marsland (101042414)
- Edmond Chow (100883365)
- Deji Sayomi (100847393)

## Information
The goal of this project is to have a fully autonomus vehicle capable of clearing snow from a designated area whilst avoiding obstacles. This will be achieved with an array of Proximity sensors to allow the vehicle to see surrounding object, and a vision sensor to keep track of when it is inside and out of bounds. The vehicle will have a unique unfolding plow which can be openand closed as needed by the vehicle.

## Example
The following image is the result of testing on Test_Map_2. Videos are included in the repo demonstrating test performance of The Plower in two of the testing maps.
![alt text](https://github.com/Alfred-Akinkoye/Snow_Plower_4805/blob/main/Testing_Map_2.png)

## Set Up
1. Download Zip File (JazzberryJamDemoSubmission.zip) (You've likely already done this)

2. Unzip the folder in your chosen directory (You've likely already done this)

3. Using a shell program, "cd" into the unzipped directory
   
    3.5 if you do not have python3 installed (The program will not run on python 2), install python3 (Current Version: 3.10.4) (https://www.python.org/downloads/)

4. Open the desired evaluation map in CoppeliaSim

5. Load the model (Plower.ttm) into the scene

6. Position "Plower_Base" to [0, -6.25, 0]. (Ensure rotations are [0, 0, 0])

7. Start the coppeliaSim simulation

8. Run plower.py in the shell program by: ("py plower.py" or "python3 plower.py")

# File Descriptions
  ## [Final Report](https://github.com/Alfred-Akinkoye/Snow_Plower_4805/blob/main/SYSC4805%20-%20L1_G1%20-%20Jazzberry%20Jam%20-%20Final%20Report.pdf)
  - The final report for the project. This is the complete document comprising the proposal, progress report, and final report documents.
  ## [PythonAPI](https://github.com/Alfred-Akinkoye/Snow_Plower_4805/tree/main/Main/PythonAPI) folder
  - This Folder contains the files required to connect to CoppeliaSim and the api's to make calls to it from python
  ## [Plower.ttm](https://github.com/Alfred-Akinkoye/Snow_Plower_4805/blob/main/Main/Plower.ttm)
  - The base model file for the vehicle
  ## [api.py](https://github.com/Alfred-Akinkoye/Snow_Plower_4805/blob/main/Main/api.py)
  - Contains additional api's we developed to make calls easier to CoppeliaSim
  ## [movementControl.py](https://github.com/Alfred-Akinkoye/Snow_Plower_4805/blob/main/Main/movementControl.py)
  - Contains all functions related to motor controls and vehicle orientation.
  ## [plower.py](https://github.com/Alfred-Akinkoye/Snow_Plower_4805/blob/main/Main/plower.py)
  - Contains the main code to run the program
  - This is where our base loop resides, and this script calls the other scripts as needed to keep our vehicle on task.
  ## [sensors.py](https://github.com/Alfred-Akinkoye/Snow_Plower_4805/blob/main/Main/sensors.py)
  - Contains all methods to check both the vision and proximity sensors.
  - Contains to classes
    - VisionSensor
    - ProximitySensor

