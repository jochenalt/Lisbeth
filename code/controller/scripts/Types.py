import pathlib
import os

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

print (URDFFilePath)
