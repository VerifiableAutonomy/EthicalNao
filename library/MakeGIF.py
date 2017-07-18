import subprocess


# folder = '/home/dieter/Dropbox/RobotLab/data/Game01_choice_True_ce_001_trial_001_mapping_001/images/'
# mask = '*.jpg'
# output = '/home/dieter/Desktop/test.gif'

# subprocess.call(["ls", "-l"])
# convert   -delay 20   -loop 0   sphere*.gif   animatespheres.gif

# command = ['convert','-delay','20','-loop','0',folder + mask,output]
# print command
# subprocess.call(command)

def MakeGIF(folder, mask, output):
    command = ['convert', '-delay', '20', '-loop', '0', folder + mask, output]
    subprocess.call(command)
