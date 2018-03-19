import numpy as np


class CalcParam():

    def __init__(self, comb_time):

        self.comb_time = comb_time


    def set_mass(self, mass_i, mass_f):
    
        d_mass = (mass_i - mass_f)
