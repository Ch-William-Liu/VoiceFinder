# plot and save

import sounddevice as sd
import soundfile as sf
import matplotlib.pyplot as plt


filename = "filename"
data , fs = sf.read(filename)


for c in range(7):
    plt.figure()
    plt.specgram(data[:,c] , 1024 , fs)
    plt.jet()
    plt.ylim([5000,25000])
    plt.clim([-125,-25])
    channel_name = "Channel-"+str(c+1)
    plt.title(channel_name)
    plt.savefig(filename+"_"+channel_name+".png")

