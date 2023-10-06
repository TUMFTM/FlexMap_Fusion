# Copyright 2023 Maximilian Leitenstern
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# ========================================== //
# Author: Maximilian Leitenstern (TUM)
# Date: 21.03.2023
# ========================================== //
#
import os

import matplotlib as mpl
import numpy as np


def load_file(dir_path, file_name, deli):
    if os.path.exists(os.path.join(dir_path, file_name)):
        var = np.transpose(np.genfromtxt(os.path.join(dir_path, file_name), delimiter=deli))
        return var
    else:
        print("Cannot find " + file_name)


def load_file_man(dir_path, file_name, deli):
    if os.path.exists(os.path.join(dir_path, file_name)):
        out = []
        lengths = []
        with open(os.path.join(dir_path, file_name), "r") as file:
            for line in file:
                line_split = line.split(deli)
                if line_split[-1] == "\n":
                    line_split.pop()
                out.append(line_split)
                lengths.append(len(line_split))
        lim = np.max(lengths)
        for li in out:
            while len(li) < lim:
                li.append("nan")
        return np.array(out, dtype=float)
    else:
        print("Cannot find " + file_name)


def load_file_str(dir_path, file_name):
    if os.path.exists(os.path.join(dir_path, file_name)):
        out = []
        with open(os.path.join(dir_path, file_name), "r") as file:
            for line in file:
                out.append(line.strip())
        return out
    else:
        print("Cannot find " + file_name)


# Set parameters for plots
def set_params():
    pgf_with_latex = {  # setup matplotlib to use latex for output
        "pgf.texsystem": "pdflatex",  # change this if using xetex or lautex
        "text.usetex": True,  # use LaTeX to write all text
        "pgf.rcfonts": False,
        "axes.labelsize": 11,
        "font.size": 11,
        "legend.fontsize": 8,  # Make the legend/label fonts
        "xtick.labelsize": 8,  # a little smaller
        "ytick.labelsize": 8,
        "pgf.preamble": "\n".join(
            [  # plots will use this preamble
                r"\usepackage[utf8]{inputenc}",
                r"\usepackage[T1]{fontenc}",
                r"\usepackage[detect-all,locale=DE]{siunitx}",
            ]
        ),
    }
    mpl.rcParams.update(pgf_with_latex)


# Define TUM color codes
def TUMcolor(name, alpha=1.0):
    if name == "Blue":
        col = [0.0 / 255.0, 101.0 / 255.0, 189.0 / 255.0, alpha]
    elif name == "Blue1":
        col = [0.0 / 255.0, 51.0 / 255.0, 89.0 / 255.0, alpha]
    elif name == "Blue2":
        col = [0.0 / 255.0, 82.0 / 255.0, 147.0 / 255.0, alpha]
    elif name == "Blue3":
        col = [100.0 / 255.0, 160.0 / 255.0, 200.0 / 255.0, alpha]
    elif name == "Blue4":
        col = [152.0 / 255.0, 198.0 / 255.0, 234.0 / 255.0, alpha]
    elif name == "Gray1":
        col = [51.0 / 255.0, 51.0 / 255.0, 51.0 / 255.0, alpha]
    elif name == "Gray2":
        col = [127.0 / 255.0, 127.0 / 255.0, 127.0 / 255.0, alpha]
    elif name == "Gray3":
        col = [204.0 / 255.0, 204.0 / 255.0, 204.0 / 255.0, alpha]
    elif name == "Ivory":
        col = [218.0 / 255.0, 215.0 / 255.0, 203.0 / 255.0, alpha]
    elif name == "Orange":
        col = [227.0 / 255.0, 114.0 / 255.0, 34.0 / 255.0, alpha]
    elif name == "Green":
        col = [162.0 / 255.0, 173.0 / 255.0, 0.0 / 255.0, alpha]
    elif name == "Black":
        col = [0.0 / 255.0, 0.0 / 255.0, 0.0 / 255.0, alpha]
    elif name == "WEBBlueDark":
        col = [7.0 / 255.0, 33.0 / 255.0, 64.0 / 255.0, alpha]
    elif name == "WEBBlueLight":
        col = [94.0 / 255.0, 148.0 / 255.0, 212.0 / 255.0, alpha]
    elif name == "WEBYellow":
        col = [254.0 / 255.0, 215.0 / 255.0, 2.0 / 255.0, alpha]
    elif name == "WEBOrange":
        col = [247.0 / 255.0, 129.0 / 255.0, 30.0 / 255.0, alpha]
    elif name == "WEBPink":
        col = [181.0 / 255.0, 92.0 / 255.0, 165.0 / 255.0, alpha]
    elif name == "WEBRed":
        col = [217.0 / 255.0, 81.0 / 255.0, 23.0 / 255.0, alpha]
    elif name == "WEBGreen":
        col = [159.0 / 255.0, 186.0 / 255.0, 54.0 / 255.0, alpha]
    else:
        col = [255.0 / 255.0, 255.0 / 255.0, 255.0 / 255.0, alpha]
    return col
