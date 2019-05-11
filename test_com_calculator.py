from k_solver.core import Actuator
import numpy as np

a = Actuator([[0, 0, 1], 'x', [0, 0, 1], 'y', [0, 0, 1]], center_of_mass_shitfts=[[0.,0.,0.5],[0.,0.,0.5],[0.,0.,0.5]], mass_parts=[1, 2, 3])

a.angles = [0, np.pi/2] # [0, 90]

print (a.com([0, 1, 2]))