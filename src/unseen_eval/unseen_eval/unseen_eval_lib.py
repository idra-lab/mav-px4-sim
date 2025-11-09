

from evo.tools import log

from evo.tools import plot
# from evo.tools.plot import PlotMode
from evo.core.metrics import PoseRelation
from evo.core.units import Unit
from evo.tools.settings import SETTINGS
from evo.core import metrics
from evo.core.units import Unit

from evo.tools import file_interface
from evo.core import sync

from evo.core import lie_algebra as lie

from evo.core.trajectory import PoseTrajectory3D

import numpy as np

import pickle



import evo.main_ape as main_ape
import evo.common_ape_rpe as common

import evo.main_rpe as main_rpe

import copy
import matplotlib as mpl

import matplotlib.pyplot as plt

import pprint

from scipy.interpolate import make_interp_spline

from scipy.signal import savgol_filter
from scipy.signal import butter, filtfilt

import seaborn as sns

from matplotlib.backends.backend_pdf import PdfPages

import cv2
import numpy as np
import glob
import os
from scipy.signal import butter, filtfilt
import matplotlib.pyplot as plt
from scipy.signal import welch, detrend

import sys
import scipy.spatial.transform as TF
import argparse

import pandas as pd


import yaml



FONTSIZE = 15
LABELPADS = 5
LABELSIZE = 10


r_CAM_DRONE = TF.Rotation.from_matrix(np.array([[0.0000000, -1.0000000,  0.0000000],
                                                [0.0000000,  0.0000000, -1.0000000],
                                                [1.0000000,  0.0000000,  0.0000000]]))

r_DRONE_CAM = r_CAM_DRONE.inv()



def collect_imported_names():
    """
    Return a sorted list of names in this module that were imported from other modules.
    Suitable to assign to __all__ (e.g. __all__ = collect_imported_names()).
    """
    current_mod = sys.modules.get(__name__)
    imported = []
    for name, val in list(globals().items()):

        # print(name,"       ", val, "\n")
        if name.startswith('_'):
            continue
        # modules themselves
        # if inspect.ismodule(val):
        #     imported.append(name)
        #     continue
        # # objects coming from a different module
        # try:
        #     obj_mod = inspect.getmodule(val)
        # except Exception:
        #     obj_mod = None
        # if obj_mod is not None and obj_mod is not current_mod:
        #     imported.append(name)
		
        imported.append(name)
    return sorted(set(imported))

__all__ = collect_imported_names()
