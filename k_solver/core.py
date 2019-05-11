"""Core features."""

from numbers import Number
from functools import reduce

import autograd.numpy as np

from .component import Link, Joint
from .solver import FKSolver, IKSolver
from .optimizer import ScipyOptimizer


class Actuator(object):
    """Represents an actuator as a set of links and revolute joints."""

    def __init__(self, tokens, optimizer=ScipyOptimizer(), center_of_mass_shitfts=None, mass_parts=None):
        """Create an actuator from specified link lengths and joint axes."""
        components = []
        for t in tokens:
            if isinstance(t, Number):
                components.append(Link([t, 0., 0.]))
            elif isinstance(t, list) or isinstance(t, np.ndarray):
                components.append(Link(t))
            elif isinstance(t, str) and t in {'x', 'y', 'z'}:
                components.append(Joint(t))
            else:
                raise ValueError(
                    'the arguments need to be '
                    'link length or joint axis: {}'.format(t)
                )

        if len(center_of_mass_shitfts) != len(mass_parts):
            raise Exception("Invalid paramters: Each center of mass position must have a mass value")
        else:
            self.mass_parts = np.array(mass_parts)
            self.total_mass = np.sum(mass_parts)
        self._fk = FKSolver(components, center_of_mass_shitfts) if center_of_mass_shitfts != None else FKSolver(components)
        self._ik = IKSolver(self._fk, optimizer)
        

        self.angles = [0.] * len(
            [c for c in components if isinstance(c, Joint)]
        )

    @property
    def angles(self):
        """The joint angles."""
        return self._angles

    @angles.setter
    def angles(self, angles):
        self._angles = np.array(angles)

    @property
    def ee(self):
        """The end-effector position."""
        return self._fk.solve(self.angles)

    def com(self, com_by_indices=None):
        com_pos_parts = self._fk.center_of_mass_parts(self.angles)
        if com_by_indices == None:
            v_mult = np.multiply(com_pos_parts.transpose(), self.mass_parts).transpose()
            com_pos = reduce(lambda f, t: f+t, v_mult)
            com_pos /= self.total_mass
            return com_pos
        else:
            coms_pos = []
            for bk in com_by_indices:
                v_mult = np.multiply(com_pos_parts[bk:].transpose(), self.mass_parts[bk:]).transpose()
                com_pos = reduce(lambda f, t: f+t, v_mult)
                com_pos /= (self.total_mass - np.sum(self.mass_parts[:bk]))
                coms_pos.append(com_pos)
            return np.array(coms_pos)   

    @ee.setter
    def ee(self, position):
        self.angles = self._ik.solve(self.angles, position)