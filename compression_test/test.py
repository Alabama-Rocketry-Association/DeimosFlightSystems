import pickle
import lz4.frame
import os
dat = 20*128*os.urandom(1024)

compressed = lz4.frame.compress(dat)
print(len(dat))
print(len(compressed))
print(len(dat)/len(compressed))
