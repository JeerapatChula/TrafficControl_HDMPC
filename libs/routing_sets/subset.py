from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import numpy as np

class subset:
    def __init__(self, arg):
        self.arg = arg;

    def length(self):
        return len(self.arg)

    def union(self, set2):
        set = subset(self.arg)
        for i in range(set2.length()):
            if set.duplicated(set2.arg[i]) == False:
                set.arg.append(set2.arg[i])
        return set

    def intersect(self, set2):
        set = []
        if self.length() > 0 and len(set2) > 0:
            #print(str(len(set2)))
            for i in range(len(set2)):
                if self.duplicated(set2[i]) == True:
                    set.append(set2[i])
        return subset(set)

    def duplicated(self, set2):
        n = self.length()
        if self.length() > 0 and set2.length() > 0:
            for i in range(n):
                if self.arg[i].arg == set2.arg:
                    return True
        return False
