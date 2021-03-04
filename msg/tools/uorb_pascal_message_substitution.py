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

toolPath = os.path.dirname(os.path.abspath(__file__))
messagesPath = os.path.abspath(os.path.join(toolPath, '../../msg'))

nonPascalMessageList = list()
convertedPascalMessageList = list()

for nonPascalMessage in os.listdir(messagesPath):
    if ('.msg' in nonPascalMessage) and (nonPascalMessage.islower() or ('_' in nonPascalMessage)):
        nonPascalBaseMessage=nonPascalMessage.replace(".msg", "")
        nonPascalMessageList.append(nonPascalBaseMessage)
        
        convertedPascalBaseMessage = capwords(nonPascalBaseMessage.replace("_", " ")).replace(" ", "")
        convertedPascalMessageList.append(convertedPascalBaseMessage)
        
        move('{:s}/{:s}.msg'.format(messagesPath,nonPascalBaseMessage), 
        	'{:s}/{:s}.msg'.format(messagesPath,convertedPascalBaseMessage))

