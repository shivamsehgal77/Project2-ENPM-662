import numpy as np
float_formatter = "{:.1f}".format
np.set_printoptions(suppress=True, formatter={'float_kind':float_formatter})

from scipy.spatial.transform import Rotation as R

from math import pi

from tqdm import tqdm

from utilities import *


with open("PoseSamples.csv", 'a') as file:

    for i in tqdm(range(1000000)):
        sample = np.random.uniform(-pi,pi,[6])
        T,_ = FKinDHParam(sample, Table_DHParam)
        Rot = T[:3, :3]
        Rot = R.from_matrix(Rot)
        euler = Rot.as_euler('zyx', degrees=True)
        line = np.array2string(sample, precision=4, separator=',', suppress_small=True).replace("[", "").replace("]", "").replace("\n", ",").replace(" ", "")+","+ \
        np.array2string(euler.flatten(), precision=4, separator=',', suppress_small=True).replace("[", "").replace("]", "").replace("\n", ",").replace(" ", "")+","+ \
        np.array2string(T[:3,3].flatten(), precision=4, separator=',', suppress_small=True).replace("[", "").replace("]", "").replace("\n", ",").replace(" ", "")+"\n"
        file.write(line)