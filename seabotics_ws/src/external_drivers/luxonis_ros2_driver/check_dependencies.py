import subprocess, shutil, os, yaml
from pathlib import Path

searchPath = "/usr/local/lib"

def LocateFile(name, path):
    for root, dirs, files in os.walk(path):
        if name in files:
            return os.path.join(root, name)
    #if search fails and file doesn't exists
    #return nothing

def CheckForMissingFiles(filesArr):
    missingFiles = []
    for file in filesArr:
        filePath = locateFile(file, searchPath)
        if not filePath.exists():
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
        filePath = Path(file)
        if not filePath.exists():
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

    missing = [CheckForMissingDep(dependencies), CheckForMissingFiles(files), CheckForMissingFiles(yamlFiles)]

    if missing:
        print("Missing dependencies", ",".join(missing))