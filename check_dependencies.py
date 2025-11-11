import subprocess, shutil, os, yaml
from pathlib import Path

searchPath = "/usr/lib/"
localPath = "/home/*/dai-ws"

def LocateFile(name, path):
    for root, dirs, files in os.walk(path):
        for f in files:
            if f.startswith(name):
                return os.path.join(root, f)
    #if search fails and file doesn't exists
    #return nothing
    return None

def CheckForMissingFiles(filesArr):
    missingFiles = []
    for file in filesArr:
        filePath = LocateFile(file, searchPath)
        if not filePath:
            missingFiles.append(file)
    return missingFiles
    
def CheckForMissingDep(depArr):
    missingDeps = []
    for dep in depArr:
        if shutil.which(dep) is None:
            missingDeps.append(dep)
    return missingDeps
    
def CheckForMissingDepYaml(filesArr):
    missingDepsYaml = []
    for file in filesArr:
        filePath = LocateFile(file, localPath)
        if not filePath:
            missingDepsYaml.append(file)
        else:
            try:
                with open(filePath, 'r') as f:
                    yaml.safe_load(f)
            except yaml.YAMLError as exc:
                print(exc)
    return missingDepsYaml
            
if __name__ == "__main__":
    files = ["libopencv.so"]
    dependencies = ["cmake", "colcon"]  
    yamlFiles = ["pcl.yaml"]

    missing = []
    missing.append(CheckForMissingFiles(files))
    missing.append(CheckForMissingDep(dependencies))
    missing.append(CheckForMissingDepYaml(yamlFiles))
    
    #Turn list of lists into a list of strings
    flatMissing = [item for sublist in missing for item in sublist]
    
    if flatMissing:
        print("Missing dependencies: " + ", ".join(flatMissing))
    else:
        print("All dependencies exist")