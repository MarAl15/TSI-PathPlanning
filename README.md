# Path Planning

Implementation of the A* and Jump Point Search (JPS) algorithms in ROS.

## Experiment 1

|![1. A*](https://github.com/MarAl15/TSI-PathPlanning/blob/main/images/1.a-star.png) | ![1. JPS](https://github.com/MarAl15/TSI-PathPlanning/blob/main/images/1.jps.png) |
|:-:|:-:|
| A* algorithm | JPS algorithm |

Path is shown in blue, expanded nodes in green and explored nodes in red.

|**Algorithm** | **Time** | **Expanded nodes** | **Distance (metres)** | **Distance (nodes)** |
|:------------:|:--------:|:------------------:|:---------------------:|:--------------------:|
|      A*      |   1.40   |         3,374      |        10.301218      |          182         |
|     JPS      |   1.00   |          433       |        11.093591      |          243         |

## Experiment 2

|![2. A*](https://github.com/MarAl15/TSI-PathPlanning/blob/main/images/2.a-star.png) | ![2. JPS](https://github.com/MarAl15/TSI-PathPlanning/blob/main/images/2.jps.png) |
|:-:|:-:|
| A* algorithm | JPS algorithm |


|**Algorithm** | **Time** | **Expanded nodes** | **Distance (metres)** | **Distance (nodes)** |
|:------------:|:--------:|:------------------:|:---------------------:|:--------------------:|
|      A*      |  191.30  |        39,552      |        37.831396      |          709         |
|     JPS      |   1.30   |          541       |        38.814111      |          751         |

## Experiment 3

|![3. A*](https://github.com/MarAl15/TSI-PathPlanning/blob/main/images/3.a-star.png) | ![3. JPS](https://github.com/MarAl15/TSI-PathPlanning/blob/main/images/3.jps.png) |
|:-:|:-:|
| A* algorithm | JPS algorithm |


|**Algorithm** | **Time** | **Expanded nodes** | **Distance (metres)** | **Distance (nodes)** |
|:------------:|:--------:|:------------------:|:---------------------:|:--------------------:|
|      A*      |  1530.20 |        72,846      |        50.038218      |          920         |
|     JPS      |   3.80   |         1,281      |          53.010       |         1,081        |


## Conclusions

For short paths there is not much difference in time and distance, although for expanded nodes there is a more significant difference.

We get noticeable improvements in search time and memory as the distance between start and end points increases, as well as the obstacles encountered along the way. The difference in distance is minimal compared to the improvements we get in time and expanded nodes.
