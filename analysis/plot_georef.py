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
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
from pyproj import Transformer
from utility import TUMcolor
from utility import load_file
from utility import load_file_man
from utility import set_params


def wgs84_to_utm32(latitude, longitude):
    transformer = Transformer.from_crs("epsg:4326", "epsg:25832", always_xy=True)
    utm_x, utm_y = transformer.transform(longitude, latitude)
    return utm_x, utm_y


class georef:
    def __init__(self):
        # Set paths
        dir_path_matching = "/home/maximilian/Documents/gitlab/autoware/output/matching"
        dir_path_traj = "/home/maximilian/Documents/data/EDGAR/Route_2"
        path_orthophoto = "/home/maximilian/Documents/data/EDGAR/Route_2/32696_5348.tif"

        # Read files
        self.source = load_file(dir_path_traj, "Route_2_GPS.txt", " ")
        self.ll = load_file_man(dir_path_matching, "lanelets_WGS84.txt", " ")

        # Load photo
        self.orthophoto = mpimg.imread(path_orthophoto)
        # Photo extent in UTM-coordinates => (left, right, bottom, top)
        # left, bottom from file name => right, top + 1000m
        self.ortho_extent = [696000, 697000, 5348000, 5349000]

    # Plot georeferenced map on orthophoto
    def plot_georef(self):
        # set_params()
        plt.figure(figsize=(8.5, 6))

        ax = plt.gca()
        ax.imshow(self.orthophoto, extent=self.ortho_extent)
        # GPS trajectory
        traj_x = []
        traj_y = []
        for line in self.source.transpose():
            utm_x, utm_y = wgs84_to_utm32(line[0], line[1])
            traj_x.append(utm_x)
            traj_y.append(utm_y)
        ax.plot(traj_x, traj_y, color=TUMcolor("Green"), linewidth=2)

        # Lanelets
        i = 0
        for line in self.ll:
            # Prepare lanelets
            lat_ll = line[::2]
            lon_ll = line[1::2]

            # Remove NaN values
            lat_ll = [val for val in lat_ll if not np.isnan(val)]
            lon_ll = [val for val in lon_ll if not np.isnan(val)]
            utm_x_ll = []
            utm_y_ll = []
            for lat, lon in zip(lat_ll, lon_ll):
                utm_x, utm_y = wgs84_to_utm32(lat, lon)
                utm_x_ll.append(utm_x)
                utm_y_ll.append(utm_y)
            ax.fill(utm_x_ll, utm_y_ll, edgecolor="w", facecolor=TUMcolor("Blue", 0.2))
            i = i + 1
        ax.set_xlabel("$X$ (Easting) in m", fontname="Arial", fontsize=11)
        ax.set_ylabel("$Y$ (Northing) in m", fontname="Arial", fontsize=11)

        ax.set_aspect("equal")
        plt.show()


# Main function - comment and uncomment functions you want to use
if __name__ == "__main__":
    # Init
    georef_ = georef()
    georef_.plot_georef()

    plt.show()
