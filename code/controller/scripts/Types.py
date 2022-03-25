import pathlib
import os
from enum import Enum

# our home directory 
#   ./description   urdf, srdf, obj and stl files
#   ./code          all source code
#   ./CAD           Inventor CAD model
HomePath=os.path.dirname(os.path.abspath(__file__)) + "/../../../"

# Path for srdf and urdf files
RobotModelPath = HomePath + "description"

# name of urdf file
URDFFileName = "solo12.urdf"
SRDFFileName = "solo.srdf"

# absolute filepath to urdf file 
URDFFilePath = RobotModelPath + "/" + URDFFileName

# absolute filepath of srdf file
SRDFFilePath = RobotModelPath + "/" + SRDFFileName
                
# All possible Gait types 
# NoMovement is still a Gait but does not move
# Static is completely static
class GaitType(Enum):
    NoGait = 0
    Pacing = 1
    Bounding = 2
    Walking = 3
    Trot = 4
    NoMovement = 5
