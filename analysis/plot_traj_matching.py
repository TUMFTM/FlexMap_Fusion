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
from matplotlib.collections import LineCollection
import matplotlib.font_manager as font_manager
import matplotlib.pyplot as plt
import numpy as np
from utility import TUMcolor
from utility import load_file
from utility import set_params


class traj_diff:
    def __init__(self, dir_path):
        # Read files
        self.source = load_file(dir_path, "source.txt", " ")
        self.target = load_file(dir_path, "target.txt", " ")
        self.target_al = load_file(dir_path, "target_al.txt", " ")
        self.target_rs = load_file(dir_path, "target_rs.txt", " ")
        self.triangles = load_file(dir_path, "triangles.txt", " ")
        self.controlPoints = load_file(dir_path, "controlPoints.txt", " ")
        self.diff_al = load_file(dir_path, "diff_al.txt", "")
        self.diff_rs = load_file(dir_path, "diff_rs.txt", "")

    # Plot initial trajectories in local coordinate system
    def plot_traj(self):
        # set_params()
        font = font_manager.FontProperties(family="Arial", style="normal", size=11)
        ax = plt.axes()
        ax.plot(
            self.source[0][0:-1:5],
            self.source[1][0:-1:5],
            label="GPS trajectory",
            color=TUMcolor("Green"),
        )
        ax.plot(
            self.target[0][0:-1:3],
            self.target[1][0:-1:3],
            label="SLAM trajectory",
            color=TUMcolor("Blue"),
        )
        ax.set_xlabel("$X$ in m", fontname="Arial", fontsize=11)
        ax.set_ylabel("$Y$ in m", fontname="Arial", fontsize=11)
        # ax.set_title('Projected Trajectories', fontname='Arial', fontsize=11)
        ax.legend(prop=font)
        ax.grid()
        ax.set_aspect("equal")
        plt.show()

    # Plot trajectories after rigid transformation with deviation
    def plot_traj_al(self):
        # set_params()
        font = font_manager.FontProperties(family="Arial", style="normal", size=11)
        fig = plt.figure()
        ax = plt.axes()
        ax.plot(
            self.source[0][0:-1:5],
            self.source[1][0:-1:5],
            label="GPS trajectory",
            color=TUMcolor("Gray2"),
        )

        diff_al_norm = plt.Normalize(self.diff_al.min(), self.diff_al.max())
        diff_al_points = self.target_al.T.reshape(-1, 1, 2)
        diff_al_segments = np.concatenate([diff_al_points[:-1], diff_al_points[1:]], axis=1)
        diff_al_lines = LineCollection(diff_al_segments, cmap="jet", norm=diff_al_norm)
        diff_al_lines.set_array(self.diff_al)
        ax.add_collection(diff_al_lines)
        cax = fig.add_axes(
            [ax.get_position().x1 - 0.1, ax.get_position().y0, 0.02, ax.get_position().height]
        )
        cbar = fig.colorbar(diff_al_lines, cax=cax)
        cbar.set_label("Deviation from GPS in m", fontname="Arial", fontsize=11)
        ax.set_xlabel("$X$ in m", fontname="Arial", fontsize=11)
        ax.set_ylabel("$Y$ in m", fontname="Arial", fontsize=11)
        ax.legend(prop=font)
        ax.grid()
        ax.set_aspect("equal")
        plt.show()

    # Plot trajectories after rubber-sheet transformation with deviation
    def plot_traj_rs(self):
        # set_params()
        font = font_manager.FontProperties(family="Arial", style="normal", size=11)
        fig = plt.figure()
        ax = plt.axes()
        ax.plot(
            self.source[0][0:-1:1],
            self.source[1][0:-1:1],
            label="GPS trajectory",
            color=TUMcolor("Gray2"),
        )
        diff_rs_norm = plt.Normalize(self.diff_rs.min(), self.diff_rs.max())
        diff_rs_points = self.target_rs.T.reshape(-1, 1, 2)
        diff_rs_segments = np.concatenate([diff_rs_points[:-1], diff_rs_points[1:]], axis=1)
        diff_rs_lines = LineCollection(diff_rs_segments, cmap="jet", norm=diff_rs_norm)
        diff_rs_lines.set_array(self.diff_rs)
        cax = fig.add_axes(
            [ax.get_position().x1 - 0.1, ax.get_position().y0, 0.02, ax.get_position().height]
        )
        ax.add_collection(diff_rs_lines)
        cbar = fig.colorbar(diff_rs_lines, cax=cax)
        cbar.set_label("Deviation from GPS in m", fontname="Arial", fontsize=11)

        ax.set_xlabel("$X$ in m", fontname="Arial", fontsize=11)
        ax.set_ylabel("$Y$ in m", fontname="Arial", fontsize=11)
        ax.legend(prop=font)
        ax.grid()
        ax.set_aspect("equal")
        plt.show()

    # Plot geometry of rubber-sheet transformation (triangles, control points)
    def plot_geom_rs(self):
        # set_params()
        font = font_manager.FontProperties(family="Arial", style="normal", size=11)
        ax = plt.axes()
        ax.plot(
            self.source[0][0:-1:3],
            self.source[1][0:-1:3],
            label="GPS trajectory",
            color=TUMcolor("Gray2"),
        )
        ax.plot(
            self.target_al[0], self.target_al[1], label="SLAM trajectory", color=TUMcolor("Blue")
        )

        for i in range(np.size(self.triangles, 1)):
            for j in range(3):
                ax.plot(
                    [self.triangles[4 * j][i], self.triangles[4 * j + 2][i]],
                    [self.triangles[4 * j + 1][i], self.triangles[4 * j + 3][i]],
                    linewidth=1,
                    color=TUMcolor("Green"),
                )
        for i in range(np.size(self.controlPoints, 1) - 4):
            ax.plot(
                [self.controlPoints[0][i], self.controlPoints[2][i]],
                [self.controlPoints[1][i], self.controlPoints[3][i]],
                marker="*",
                color=TUMcolor("Orange"),
            )

        ax.set_xlabel("$X$ in m", fontname="Arial", fontsize=11)
        ax.set_ylabel("$Y$ in m", fontname="Arial", fontsize=11)
        ax.legend(loc="upper left", prop=font)
        ax.grid()
        ax.set_aspect("equal")
        plt.show()

    # Statistical values on deviations
    def print_statistics(self):
        rmse_al = np.sqrt(np.mean(self.diff_al**2))
        rmse_rs = np.sqrt(np.mean(self.diff_rs**2))

        print("RMSE aligned target trajectory: " + str(rmse_al))
        print("RMSE rubber-sheeted target trajectory: " + str(rmse_rs))
        print("Mean aligned target trajectory: " + str(np.mean(self.diff_al)))
        print("Mean rubber-sheeted target trajectory: " + str(np.mean(self.diff_rs)))
        print("Median aligned target trajectory: " + str(np.median(self.diff_al)))
        print("Median rubber-sheeted target trajectory: " + str(np.median(self.diff_rs)))
        print("Standard deviation aligned target trajectory: " + str(np.std(self.diff_al)))
        print("Standard deviation rubber-sheeted target trajectory: " + str(np.std(self.diff_rs)))


# Main function - comment and uncomment functions you want to use
if __name__ == "__main__":
        # Check command-line arguments
    if len(sys.argv) != 2:
        print("Usage: plot_traj_matching.py <path/to/output/traj_matching>")
        sys.exit(1)

    # Get command-line arguments
    dir_path = sys.argv[1]
    # Init
    traj = traj_diff(dir_path)

    traj.plot_traj()
    traj.plot_traj_al()
    traj.plot_traj_rs()
    traj.plot_geom_rs()
    traj.print_statistics()

    plt.show()
