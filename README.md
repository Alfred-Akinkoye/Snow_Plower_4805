# Snow_Plower_4805
Autonomous Snow Plow Project for L1 Group 1 of SYSC 4805
# Team Members
- Alfred Akinkoye (101109483)
- Michael Marsland (101042414)
- Edmond Chow (100883365)
- Deji Sayomi (100847393)

## Information
We will be using this github to share both the CoppeliaSim scenes (Building the robot, the plow, sensors, motors, etc...)<br><br>
We will also be including the Python code to drvie the autonomous plow through the CoppeliaSim Python API<br><br>
  
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
  ## [Python API](https://github.com/Alfred-Akinkoye/Snow_Plower_4805/tree/main/PythonAPI) folder
      - This Folder contains the files required to connect to CoppeliaSim and the api's to make calls to it from python
  ## [Plower.ttm](https://github.com/Alfred-Akinkoye/Snow_Plower_4805/blob/main/Plower.ttm)
      - The base model file for the vehicle
  ## [API.py](https://github.com/Alfred-Akinkoye/Snow_Plower_4805/blob/main/api.py)
      - Contains additional api's we developed to make calls easier to CoppeliaSim
  ## [MovementControls.py](https://github.com/Alfred-Akinkoye/Snow_Plower_4805/blob/main/movementControl.py)
      - Contains all functions related to motor controls and vehicle orientation.
  ## [PLower.py](https://github.com/Alfred-Akinkoye/Snow_Plower_4805/blob/main/plower.py)
      - Contains the main code to run the program
      - This is where our base loop resides, and this script calls the other scripts as needed to keep our vehicle on task.
  ## [Sensors.py](https://github.com/Alfred-Akinkoye/Snow_Plower_4805/blob/main/sensors.py)
      - Contains all methods to check both the vision and proximity sensors.
      - Contains to classes
        - VisionSensor
        - ProximitySensor
  ## [Project Proposal](https://github.com/Alfred-Akinkoye/Snow_Plower_4805/blob/main/SYSC4805%20-%20L1_G1%20-%20Jazzberry%20Jam%20-%20Project%20Proposal.pdf)
      - Contains information of our base plan on how to complete this project
      - And what we hope to accomplish

