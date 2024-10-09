from os.path import exists
from os import mkdir

class cachedDataType:
        
    cachedDemoName = ""
    cachedCfgPath = ""
    cachedDeviceName = ""

    def __init__(self):
        try:
            if(exists("history\cachedData.txt")):
                configHistoryFile = open("history\cachedData.txt", 'r')
                lines = configHistoryFile.readlines()
                self.cachedDeviceName = lines[0][0:-1]
                self.cachedDemoName = lines[1][0:-1]
                self.cachedCfgPath = lines[2]
                configHistoryFile.close()
        except:
            print("No cached data")

    def writeToFile(self):
        if not exists("history"):
        # Note that this will create the folder in the caller's path, not necessarily in the Industrial Viz Folder
            mkdir("history")
        configHistoryFile = open("history\cachedData.txt", 'w')
        configHistoryFile.write(self.cachedDeviceName + '\n')
        configHistoryFile.write(self.cachedDemoName + '\n')
        configHistoryFile.write(self.cachedCfgPath)
        configHistoryFile.close()

    def getCachedDeviceName(self):
        return self.cachedDeviceName

    def getCachedDemoName(self):
        return self.cachedDemoName

    def getCachedCfgPath(self):
        return self.cachedCfgPath

    def setCachedDemoName(self, newDemo):
        self.cachedDemoName = newDemo
        self.writeToFile()

    def setCachedDeviceName(self, newDevice):
        self.cachedDeviceName = newDevice
        self.writeToFile()

    def setCachedCfgPath(self, newPath):
        self.cachedCfgPath = newPath
        self.writeToFile()

