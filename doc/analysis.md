# Analysis

## Overview

- matching statistics shown during execution of tool
- export of various data by setting corresponding parameters in config-file
  - data is exported to `.txt` files that are then read by python-scripts
  - set export path in config-file
  - adjust import paths at the beginning of python-scripts
- analysis scripts in `/analysis`

### 1. Analysis of Geometric Alignment

- visualization of [UTM-projection](https://apps.dtic.mil/sti/pdfs/ADA266497.pdf), [Umeyama transformation](https://web.stanford.edu/class/cs273/refs/umeyama.pdf) and [Rubber-Sheet transformation](https://www.tandfonline.com/doi/abs/10.1559/152304085783915135)
- execute script `plot_traj_matching.py` in `/analysis`
- produces graphs shown in [alignment](alignment.md)
- calculation of deviation between trajectories based on distance between point and line segment
- distance from point $\vec{p}$ on target trajectory to segment of points $\vec{a}$ and $\vec{b}$ on master:

```math
d =
    \begin{cases}
    \|\vec{p} - \vec{a}\| & \quad u < 0. \\
    \|\vec{p} - \vec{b}\| & \quad u > 1. \\
    \frac{|(\vec{b} - \vec{a}) \times (\vec{p} - \vec{a})|}{\|\vec{b} - \vec{a}\|} & \quad 0 \leq u \leq 1.
    \end{cases}
```

where

```math
u = \frac{(\vec{b} - \vec{a}) \cdot (\vec{p} - \vec{a})}{\|\vec{b} - \vec{a}\|}
```

- minimum deviation calculated by minimum distance of $\vec{p}$ to any segment of subsequent points
- note that this produces falsified results near intersections of the trajectories

### 2. Analysis of Matching/Conflation

- command window
  - matching statistics shown in command window during execution indicate
  - geometric similarity measures calculated as in [matching algorithm](matching.md).
  - mean values of $\beta$, $l$, $d$, $\bar{S}$ and $score$
- script `plot_matching.py` in `/analysis`

  - (un)comment the desired modules in the main function
  - calculation of similarity measures and visualization
  - visualization of matching results (match polylines, linkages, original lanelets, updated lanelets)
  - calculation of precision and recall

    ```math
    precision = \frac{True~Positive}{True~Positive + False~Positive} \\[5pt]%
    recall = \frac{True~Positive}{True~Positive + False~Negative}
    ```

    | Prediction/Reality |                                             True                                              |                                                 False                                                  |
    | :----------------: | :-------------------------------------------------------------------------------------------: | :----------------------------------------------------------------------------------------------------: |
    |      Positive      | Algorithm correctly identifies the corresponding OSM polyline for a given reference polyline. | Algorithm predicts a match but the prediction is wrong (either wrong OSM polyline or no match at all). |
    |      Negative      |     Algorithm correctly identifies that there is no match for a given reference polyline.     |     Algorithm does not identify a match for a given reference polyline when in fact there is one.      |

  - match considered True Positive if $score$ > 0.8, otherwise False Positive
  - identification of false negatives by manual investigation (use module `identify_fn()` inside `plot_matching.py`)

### 3. Analysis of Georeferencing

- visualize lanelet map and GNSS trajectory in comparison to orthophoto
- orthophotos from Bavaria available on [OPENDATA](https://geodaten.bayern.de/opengeodata/OpenDataDetail.html?pn=dop40)
- orthophotos are rectified in UTM32 (EPSG:25832) coordinates and have a size of 1km x 1km each
- script `plot_georef.py` in `/analysis`
  - download photos from desired area on [OPENDATA](https://geodaten.bayern.de/opengeodata/OpenDataDetail.html?pn=dop40)
  - adjust path of image and set photo extent (boundaries of photo in UTM-coordinates)
