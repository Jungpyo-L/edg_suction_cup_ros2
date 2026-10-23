#!/usr/bin/env python
from ast import arg
import os
from datetime import datetime
import numpy as np
import pandas as pd
from scipy.io import savemat
import re

class fileSaveHelp(object):
    def __init__(self, savingFolderName = 'EDG_Experiment'):
        self.savingFolderName = savingFolderName
        self.ResultSavingDirectory = os.path.expanduser('~') + '/' + self.savingFolderName + '/' + datetime.now().strftime("%y%m%d")        
        if not os.path.exists(self.ResultSavingDirectory):
            os.makedirs(self.ResultSavingDirectory)
        
    def getLastMatFileSaved(self):    
        fileList = []
        for file in os.listdir(self.ResultSavingDirectory):
            if file.endswith(".mat"):
                fileList.append(file)
        try:
            return sorted(fileList)[-1]
        except Exception as e:
            print(e)
        return "none"
    
    # The csv files in /tmp that this user's logger wrote. /tmp is shared, and
    # another account's leftover logs can be neither removed nor moved by us,
    # so they are skipped rather than aborting the run.
    def tmpCsvFiles(self):
        fileList = []
        for file in os.listdir("/tmp"):
            fileName = os.path.join("/tmp", file)
            if file.endswith(".csv") and os.stat(fileName).st_uid == os.getuid():
                fileList.append(fileName)
        return fileList

    # Clear all csv files in the tmp folder
    def clearTmpFolder(self):
        for fileName in self.tmpCsvFiles():
            os.remove(fileName)


    def saveDataParams(self, args=None, appendTxt = ''):

        #check if CSV Files are available
        tmp_dummyFolder = '/tmp/processed_csv'
        if os.path.exists(tmp_dummyFolder) and not os.access(tmp_dummyFolder, os.W_OK):
            tmp_dummyFolder = '/tmp/processed_csv_%d' % os.getuid()
        if not os.path.exists(tmp_dummyFolder):
            os.makedirs(tmp_dummyFolder)

        # First Check all CSV files in /tmp/ and bring them in as a variable
        fileList = self.tmpCsvFiles()

        print("csv files: ", fileList)
        if not fileList:
            raise RuntimeError(
                "No CSV files found in /tmp. Start data_logger.py first, make sure "
                "TopicsList.txt topics are currently published, and check that the "
                "data_logging service created output files."
            )

        print("grabbing columns from csv files into one dataframe")
        savingDictionary = {}
        errorCount = 0
        processedFileCount = 0
        firstFileName = fileList[0]
        for fileName in fileList:
            print("trying file: ", fileName)
            try:    
                df=pd.read_csv(fileName)             
                                    
                thisColumnName = df.columns.tolist()
                
                splitedList = re.split(r'_|\.', fileName)        
                thisTopicName = ''.join(splitedList[4:-1])        
                
                savingDictionary[thisTopicName+"_columnName"] = thisColumnName
                savingDictionary[thisTopicName+"_data"] = df.values
                processedFileCount += 1
                #move to temparary folder    
                os.rename(fileName, tmp_dummyFolder + '/' + re.split('/',fileName)[-1])
            except Exception as e:
                print(e)
                errorCount +=1

        if errorCount > 0:
            print("!!!!-- Mised ", errorCount, " csv files --!!!!")

        if processedFileCount == 0:
            raise RuntimeError(
                "CSV files were created, but none contained readable data. Keep "
                "logging enabled until at least one message arrives on a configured topic."
            )

        # Save all the contents in the args as variables
        if args is not None:
            argsDic = vars(args)
            for key in list(argsDic.keys()):
                # savemat cannot write None, and an optional argument that was
                # not given is exactly that - it would abort the save after the
                # whole run had already been performed. Store an empty string so
                # the key is still there: dropping it instead would make an
                # unused option indistinguishable from an older run predating it.
                value = argsDic[key]
                savingDictionary[key] = "" if value is None else value
        

        fileNameParts = re.split(r'_|\.', firstFileName)
        savingFileName_noDir = 'DataLog_'+ '_'.join(fileNameParts[1:4])
        savingFileName = self.ResultSavingDirectory + '/' + savingFileName_noDir + '_' + appendTxt + '.mat'
        print(savingFileName)

        savemat(savingFileName, savingDictionary)
        print("savingFileName_noDir: ", savingFileName_noDir)

        return self.ResultSavingDirectory, savingFileName_noDir


