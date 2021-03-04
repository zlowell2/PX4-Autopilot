#!/usr/bin/env python3

# Converts uORB messages to pascal case and replaces their respective includes.
# Do not run unless you fully understand that this will completely change all your messages.
# License: Apache 2.0
# Author: Benjamin Perseghetti
# Author Email: bperseghetti@rudislabs.com
# Maintainer: Benjamin Perseghetti
# Maintainer Email: bperseghetti@rudislabs.com

from string import capwords
import os
import sys
from shutil import move
import subprocess
import shlex

neverSetToTrue = False
toolPath = os.path.dirname(os.path.abspath(__file__))
messagesPath = os.path.abspath(os.path.join(toolPath, '../../msg'))
changePath = os.path.abspath(os.path.join(toolPath,'../../../PX4-Autopilot'))
changeFolders = ['boards/', 'Documentation/', 'launch/',  
                 'msg/', 'posix-configs/', 'src/', 
                 'test_data/', 'validation/', 'cmake/', 
                 'integrationtests/', 'mavlink/', 'platforms/', 
                 'ROMFS/', 'test/', 'Tools/']

nonPascalMessageList = list()
convertedPascalMessageList = list()

for nonPascalMessage in os.listdir(messagesPath):
    if ('.msg' in nonPascalMessage) and (nonPascalMessage.islower() or ('_' in nonPascalMessage)):
        nonPascalBaseMessage = nonPascalMessage.replace(".msg", "")
        nonPascalMessageList.append(nonPascalBaseMessage)
        
        convertedPascalBaseMessage = capwords(
            nonPascalBaseMessage.replace("_", " ")).replace(" ", "")
        convertedPascalMessageList.append(convertedPascalBaseMessage)
        
        move('{:s}/{:s}.msg'.format(messagesPath,nonPascalBaseMessage), 
            '{:s}/{:s}.msg'.format(messagesPath,convertedPascalBaseMessage))

        if neverSetToTrue:
            print('\nRunning changes for: {:s} to {:s}'.format(
                    nonPascalBaseMessage, convertedPascalBaseMessage))
            for folder in changeFolders:
                cmdGrep='grep -rli \'{:s}/{:s}\' -e "{:s}.h" *'.format(
                    changePath, folder, nonPascalBaseMessage)
                cmdGrepPopen=shlex.split(cmdGrep)
                grepPopen = subprocess.Popen(cmdGrepPopen, stdout=subprocess.PIPE, 
                    stderr=subprocess.PIPE, text=True)
                grepOut, grepErr = grepPopen.communicate()
                grepPopen.wait()
                grepOut=list(grepOut.split("\n"))

                for modifyFile in grepOut:
                    if modifyFile != '':
                        print('Modifying file: {:s}'.format(modifyFile))
                        cmdSed='sed -i "s/{:s}/{:s}/g" {:s}'.format(
                            nonPascalBaseMessage,convertedPascalBaseMessage,modifyFile)
                        cmdSedPopen=shlex.split(cmdSed)
                        sedPopen = subprocess.Popen(cmdSedPopen, stdout=subprocess.PIPE, 
                            stderr=subprocess.PIPE, text=True)
                        sedOut, sedErr = sedPopen.communicate()
                        sedPopen.wait()

