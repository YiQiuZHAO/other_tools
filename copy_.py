from glob import glob
import os
from shutil import copyfile

files = glob('data-seg/*.txt')
savepath = './data-seg/'

for file in files:
    # c = 'xcopy '+file + ' ' + savepath
    # os.system(c)
    name = os.path.splitext(os.path.split(file)[1])[0]
    filename = name + '.jpg'
    copyfile('save/'+filename,savepath+filename)