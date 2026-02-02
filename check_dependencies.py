import subprocess, shutil, os, yaml
from pathlib import Path

searchPath = "/usr/lib/"
localPath = Path(__file__).resolve().parent

def LocateFile(name, path):
    for root, dirs, files in os.walk(path):
        for f in files:
            if f.startswith(name):
                return os.path.join(root, f)
    #if search fails and file doesn't exist
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
    invalidYaml = []
    for file in filesArr:
        filePath = LocateFile(file, localPath)

        #File not found
        if not filePath:
            missingDepsYaml.append(file)
            continue

        #File found - validate YAML
        else:
            try:
                with open(filePath, 'r') as f:
                    yaml.safe_load(f)

            except yaml.YAMLError as exc:
                print(exc)
                invalidYaml.append(file)

            except OSError as exc:
                print(exc)

    #Return list with missing files and invalid yaml files
    return missingDepsYaml#, invalidYaml
            
if __name__ == "__main__":
    files = ["libopencv_"]
    dependencies = ["cmake", "colcon"]  
    yamlFiles = ["pcl.yaml"]

    missing = [CheckForMissingFiles(files),
               CheckForMissingDep(dependencies),
               CheckForMissingDepYaml(yamlFiles)
               ]

    #Turn list of lists into a list of strings
    flatMissing = [item for sublist in missing for item in sublist]
    
    if flatMissing:
        print("Missing dependencies: " + ", ".join(flatMissing))
    else:
        print("All dependencies exist")