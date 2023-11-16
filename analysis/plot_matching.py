#!/usr/bin/env python3
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
import sys
import matplotlib.pyplot as plt
import numpy as np
from utility import TUMcolor
from utility import load_file
from utility import load_file_man
from utility import load_file_str
from utility import set_params


class matching:
    def __init__(self, dir_path):
        # Read files
        self.reference = load_file_man(dir_path, "reference_plines.txt", " ")
        self.target = load_file_man(dir_path, "target_plines.txt", " ")
        self.buffers = load_file_man(dir_path, "buffers.txt", " ")
        self.osm = load_file_man(dir_path, "osm_linestrings.txt", " ")
        self.ll = load_file_man(dir_path, "lanelets.txt", " ")
        self.ll_col = load_file_str(dir_path, "lanelets_colors.txt")
        self.ll_new = load_file_man(dir_path, "lanelets_updated.txt", " ")
        self.conn = load_file_man(dir_path, "match_conn.txt", " ")
        self.dAng = load_file(dir_path, "dAng.txt", "")
        self.dLen = load_file(dir_path, "dLen.txt", "")
        self.dChord = load_file(dir_path, "dChord.txt", "")
        self.dPoly = load_file(dir_path, "dPoly.txt", "")
        self.dChamfer = load_file(dir_path, "dChamfer.txt", "")
        self.lenRefPline = load_file(dir_path, "lenRefPline.txt", "")
        self.score = load_file(dir_path, "score.txt", "")

    # Plot matching results with lanelets and osm road network
    def plot_matching(self):
        # set_params()
        plt.figure(figsize=(8.5, 6))
        ax = plt.gca()
        cols = [
            "WEBBlueDark",
            "WEBBlueLight",
            "WEBYellow",
            "WEBOrange",
            "WEBPink",
            "WEBRed",
            "WEBGreen",
        ]
        # Lanelets
        i = 0
        for line in self.ll:
            # Prepare lanelets
            x_ll = line[::2]
            y_ll = line[1::2]

            # Remove NaN values
            x_ll = [val for val in x_ll if not np.isnan(val)]
            y_ll = [val for val in y_ll if not np.isnan(val)]
            ax.fill(x_ll, y_ll, edgecolor="k", facecolor=TUMcolor(self.ll_col[i], 0.2))
            i = i + 1
        # Updated lanelets
        """ for line in self.ll_new:
            # Prepare lanelets
            x_ll_new = line[::2]
            y_ll_new = line[1::2]

            # Remove NaN values
            x_ll_new = [val for val in x_ll_new if not np.isnan(val)]
            y_ll_new = [val for val in y_ll_new if not np.isnan(val)]
            ax.fill(x_ll_new, y_ll_new, edgecolor='k', facecolor=TUMcolor('Blue', 0.2)) """
        # Buffers
        """ for line in self.buffers:
            x_buf = line[::2]
            y_buf = line[1::2]
            # Remove NaN values
            x_buf = [val for val in x_buf if not np.isnan(val)]
            y_buf = [val for val in y_buf if not np.isnan(val)]
            ax.fill(x_buf, y_buf, edgecolor='0.7', facecolor=TUMcolor('Ivory', 0.3)) """
        # OSM network
        for line in self.osm:
            # Prepare OSM network
            x_osm = line[::2]
            y_osm = line[1::2]

            # Remove NaN values
            x_osm = [val for val in x_osm if not np.isnan(val)]
            y_osm = [val for val in y_osm if not np.isnan(val)]
            ax.plot(x_osm, y_osm, color=TUMcolor("Orange", 0.5), linewidth=4)
        # Reference polylines
        j = 0
        for line in self.reference:
            i = 0
            while ((4 * i)) < len(line) and not np.isnan(line[4 * i]):
                ax.arrow(
                    line[4 * i],
                    line[4 * i + 1],
                    line[4 * i + 2] - line[4 * i],
                    line[4 * i + 3] - line[4 * i + 1],
                    length_includes_head=True,
                    width=0.15,
                    color=TUMcolor(cols[j]),
                )
                i += 1
            j = j + 1
            if j == len(cols):
                j = 0
        # Target polylines
        j = 0
        for line in self.target:
            i = 0
            while ((4 * i)) < len(line) and not np.isnan(line[4 * i]):
                ax.arrow(
                    line[4 * i],
                    line[4 * i + 1],
                    line[4 * i + 2] - line[4 * i],
                    line[4 * i + 3] - line[4 * i + 1],
                    length_includes_head=True,
                    width=0.15,
                    color=TUMcolor(cols[j]),
                )
                i += 1
            j = j + 1
            if j == len(cols):
                j = 0
        # Match linkages
        for line in self.conn:
            i = 0
            while ((4 * i)) < len(line) and not np.isnan(line[4 * i]):
                ax.arrow(
                    line[4 * i],
                    line[4 * i + 1],
                    line[4 * i + 2] - line[4 * i],
                    line[4 * i + 3] - line[4 * i + 1],
                    length_includes_head=True,
                    width=0.1,
                    color=TUMcolor("Green"),
                )
                i += 1

        ax.set_xlabel("$X$ in m", fontname="Arial", fontsize=11)
        ax.set_ylabel("$Y$ in m", fontname="Arial", fontsize=11)
        ax.grid()
        ax.set_aspect("equal")
        plt.show()

    # Function to identify false negative matches (plots every unmatched reference polyline
    # with the whole osm road network => user investigates if it is a false negative =>
    # close the plot => answer the prompt)
    def identify_fn(self):
        # set_params()
        # Get indices of non-matches (score = 0) that are longer than the threshold
        ind_ref_pline = np.where(self.lenRefPline >= 1.5)
        self.reference_fil = self.reference[ind_ref_pline]
        ind_score = np.where(self.score[ind_ref_pline] == 0)
        self.reference_fil_ = self.reference_fil[ind_score]
        count_fn = 0

        # Plot corresponding non-matches along with the osm network
        for line in self.reference_fil_:
            ax = plt.axes()
            i = 0
            while ((4 * i)) < len(line) and not np.isnan(line[4 * i]):
                ax.arrow(
                    line[4 * i],
                    line[4 * i + 1],
                    line[4 * i + 2] - line[4 * i],
                    line[4 * i + 3] - line[4 * i + 1],
                    length_includes_head=True,
                    width=0.08,
                    color=TUMcolor("Blue"),
                )
                i += 1

            for point in self.osm:
                # Prepare OSM network
                x_osm = point[::2]
                y_osm = point[1::2]

                # Remove NaN values
                x_osm = [val for val in x_osm if not np.isnan(val)]
                y_osm = [val for val in y_osm if not np.isnan(val)]
                ax.plot(x_osm, y_osm, color=TUMcolor("Orange"))
            ax.set_xlabel("$X$ in m", fontname="Arial", fontsize=11)
            ax.set_ylabel("$Y$ in m", fontname="Arial", fontsize=11)
            ax.set_title("Unmatched polyline with OSM network", fontname="Arial", fontsize=11)
            ax.grid()
            ax.set_xlim(line[0] - 20, line[0] + 20)
            ax.set_ylim(line[1] - 20, line[1] + 20)
            ax.set_aspect("equal")
            plt.show()
            curr = input("Is the current plot a false negative? (yes or no): ")
            if curr == "yes":
                count_fn = count_fn + 1
        print("Identified false negatives " + str(count_fn))

    # Plot the length of the reference polylines over the amount of matches
    def plot_len_ref_pline(self):
        # set_params()
        ax = plt.axes()
        ax.plot(
            range(1, len(self.lenRefPline), 1),
            self.lenRefPline[0:-1:1],
            "*",
            color=TUMcolor("Blue"),
        )
        ax.set_xlabel("Match index", fontname="Arial", fontsize=11)
        ax.set_ylabel("$l$ in m", fontname="Arial", fontsize=11)
        ax.grid()
        plt.show()

    # Plot geometric similarity measures over matches
    def plot_geo_measures(self):
        # set_params()
        # Angles
        ax = plt.axes()
        ax.plot(
            range(1, len(self.dAng), 1),
            self.dAng[0:-1:1],
            "*",
            label="Angle",
            color=TUMcolor("Blue"),
        )
        ax.set_xlabel("Match index", fontname="Arial", fontsize=11)
        ax.set_ylabel("$angle$ in rad", fontname="Arial", fontsize=11)
        ax.legend(loc="upper left")
        ax.grid()
        plt.show()

        # Length and Chord
        ax = plt.axes()
        ax.plot(
            range(1, len(self.dLen), 1),
            self.dLen[0:-1:1],
            "*",
            label="Length",
            color=TUMcolor("Green"),
        )
        ax.plot(
            range(1, len(self.dChord), 1),
            self.dChord[0:-1:1],
            "*",
            label="Chord",
            color=TUMcolor("Orange"),
        )
        ax.set_xlabel("Match index", fontname="Arial", fontsize=11)
        ax.set_ylabel("$l$ in m", fontname="Arial", fontsize=11)
        ax.legend(loc="upper left")
        ax.grid()
        plt.show()

        # Area of combined polygon divided by sum of lengths
        ax = plt.axes()
        ax.plot(
            range(1, len(self.dPoly), 1),
            self.dPoly[0:-1:1],
            "*",
            label="Poly Area",
            color=TUMcolor("Gray1"),
        )
        ax.set_xlabel("Match index", fontname="Arial", fontsize=11)
        ax.set_ylabel("$|S/(L_{ref} + L_{target})|$ in m", fontname="Arial", fontsize=11)
        ax.grid()
        plt.show()

        # Chamfer distance
        ax = plt.axes()
        ax.plot(
            range(1, len(self.dChamfer), 1),
            self.dChamfer[0:-1:1],
            "*",
            label="Chamfer distance",
            color=TUMcolor("Green"),
        )
        ax.set_xlabel("Match index", fontname="Arial", fontsize=11)
        ax.set_ylabel("Chamfer distance in m", fontname="Arial", fontsize=11)
        ax.grid()
        plt.show()

        # Score
        ax = plt.axes()
        ax.plot(
            range(1, len(self.score), 1),
            self.score[0:-1:1],
            "*",
            label="Score",
            color=TUMcolor("Blue"),
        )
        ax.set_xlabel("Match index", fontname="Arial", fontsize=11)
        ax.set_ylabel("score", fontname="Arial", fontsize=11)
        ax.grid()
        plt.show()

    # Print matching statistics (if you want the correct value for recall,
    # identify the amount of false negatives first => see function above,
    # otherwise, enter a random value)
    def print_statistics(self):
        fn = int(input("Enter the amount of false negatives: "))
        print("Total amount of polylines: " + str(len(self.reference)))
        print("Mean length of reference polyline: " + str(np.mean(self.lenRefPline)))
        # Get indices where reference polyline is longer than 1.5m
        ind_ref_pline = np.where(self.lenRefPline >= 1.5)
        self.d_ang_fil = self.dAng[ind_ref_pline]
        self.d_len_fil = self.dLen[ind_ref_pline]
        self.d_chord_fil = self.dChord[ind_ref_pline]
        self.d_poly_fil = self.dPoly[ind_ref_pline]
        self.d_chamfer_fil = self.dChamfer[ind_ref_pline]
        self.score_fil = self.score[ind_ref_pline]

        print(
            "Polylines filtered out because of length threshold: "
            + str(len(self.score) - len(self.score_fil))
        )

        # Filter out non-matches (where measure = 0)
        self.d_ang_fil_ = np.where(self.d_ang_fil != 0, self.d_ang_fil, np.nan)
        self.d_len_fil_ = np.where(self.d_len_fil != 0, self.d_len_fil, np.nan)
        self.d_chord_fil_ = np.where(self.d_chord_fil != 0, self.d_chord_fil, np.nan)
        self.d_poly_fil_ = np.where(self.d_poly_fil != 0, self.d_poly_fil, np.nan)
        self.d_chamfer_fil_ = np.where(self.d_chamfer_fil != 0, self.d_chamfer_fil, np.nan)
        self.score_fil_ = np.where(self.score_fil != 0, self.score_fil, np.nan)

        print("Polylines without match: " + str(np.sum(np.isnan(self.score_fil_))))
        print("Mean angle between matches: " + str(np.nanmean(self.d_ang_fil_)))
        print("Mean length difference between whole matches: " + str(np.nanmean(self.d_len_fil_)))
        print("Mean chord difference between whole matches: " + str(np.nanmean(self.d_chord_fil_)))
        print("Mean S difference between whole matches: " + str(np.nanmean(self.d_poly_fil_)))
        print(
            "Mean chamfer distance between whole matches: " + str(np.nanmean(self.d_chamfer_fil_))
        )
        print("Mean score matches: " + str(np.nanmean(self.score_fil_)))
        total = np.count_nonzero(self.score_fil)
        tp = np.sum(self.score_fil >= 0.8)
        print("Matching precision: " + str(tp / total))
        print("Matching recall: " + str(tp / (total + fn)))


# Main function - comment and uncomment functions you want to use
if __name__ == "__main__":
    # Check command-line arguments
    if len(sys.argv) != 2:
        print("Usage: plot_matching.py <path/to/output/matching>")
        sys.exit(1)

    # Get command-line arguments
    dir_path = sys.argv[1]
    # Init
    matching_ = matching(dir_path)
    matching_.plot_matching()
    # matching_.identify_fn()
    matching_.plot_len_ref_pline()
    matching_.plot_geo_measures()
    matching_.print_statistics()

    plt.show()
