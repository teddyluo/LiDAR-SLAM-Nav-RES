# Google Cartographer--《Real-Time Loop Closure in 2D LIDAR SLAM》 阅读笔记

作者: Wolfgang Hess, Damon Kohler, Holger Rapp, Danie Andor
校对: teddyluo, huiwu.luo@aliyun.com

## 摘要
Portable laser range-finders, further referred to as LIDAR, and simultaneous localization and mapping (SLAM) are an efficient method of acquiring as-built floor plans. Generating and visualizing floor plans in real-time helps the operator assess the quality and coverage of capture data. Building a portable capture platform necessitates operating under limited computational resources. We present the approach used in our backpack mapping platform which achieves real-time mapping and loop closure at a 5 cm resolution. To achieve realtime loop closure, we use a branch-and-bound approach for computing scan-to-submap matches as constraints. We provide experimental results and comparisons to other well known approaches which show that, in terms of quality, our approach is competitive with established techniques.
便携式激光测距仪(简称为LIDAR)搭配同步定位与建图（SLAM）是建立平面地图的比较有效的一种方法。实时生成和绘制平面图能很好地辅助操作者评估捕获数据的质量和覆盖范围。因此，在有限的计算资源条件下建立便携式的数据捕获平台是非常有必要的。我们提供了一种用于背包(backpack)平台下的建图方法，能够实时绘制5cm精度的地图及进行闭环检验。为了让回环检测实时计算，我们使用了分支上界法(branch-and-bound)，将scan-to-map的匹配作为其约束条件。我们提供了实验结果，与其他非常著名的方法相比，我们的方法在地图构建质量上可以与他们相匹敌。

## Introduction
As-built floor plans are useful for a variety of applications. Manual surveys to collect this data for building management tasks typically combine computed-aided design (CAD) with laser tape measures. These methods are slow and, by employing human preconceptions of buildings as collections of straight lines, do not always accurately describe the true nature of the space. Using SLAM, it is possible to swiftly and accurately survey buildings of sizes and complexities that would take orders of magnitude longer to survey manually.
建立平面地图对于很多应用都是非常有用的。收集这种数据进行管理工作的典型人工调研方法需要结合计算辅助设计(CAD)和激光卷尺(laser tape measures)的测量结果。这些方法通常过程很漫长，并且在测量建筑物直线过程中引入人类的测量偏见，并不能精确描述空间的真实性质。SLAM技术可以快速准确地调查建筑物的大小尺寸和复杂程度，而同样的工作采用人工调查则需要更长的周期。

Applying SLAM in this field is not a new idea and is not the focus of this paper. Instead, **the contribution of this paper is a novel method for reducing the computational requirements of computing loop closure constraints from laser range data**. This technique has enabled us to map very large floors, tens-of-thousands of square meters, while providing the operator fully optimized results in real-time.
在这个领域中使用SLAM技术并不是一件新鲜的事，也不是本文的重点。相反，__本文的贡献在于提出一种新方法，它从雷达数据计算回环检测过程中降低了计算量__。这种技术能够帮助我们绘制数十万平方米的非常大的楼层的地图，同时为操作员提供实时全面优化的结果。

## Relative Work
Scan-to-scan matching is frequently used to compute relative pose changes in laser-based SLAM approaches, for example [1]–[4]. On its own, however, scan-to-scan matching quickly accumulates error.
扫描到扫描(scan-to-scan)的匹配法常常用在基于激光SLAM的方法中计算相对位姿变化，例如[1]-[4]。本身而言，扫描到扫描的匹配会很快积累误差。

Scan-to-map matching helps limit this accumulation of error. One such approach, which uses Gauss-Newton to find local optima on a linearly interpolated map, is [5]. In the presence of good initial estimates for the pose, provided in this case by using a sufficiently high data rate LIDAR, locally optimized scan-to-map matching is efficient and robust. On unstable platforms, the laser fan is projected onto the horizontal plane using an inertial measurement unit (IMU) to estimate the orientation of gravity.
扫描到地图(scan-to-map)的匹配法有助于减少这种误差。其中的一种方法，使用高斯牛顿法在线性插值图中寻找出局部最优解，它就是文献[5]采用的方法。存在良好的位姿初始估计值的情况下，例如本文情况为通过使用频率足够高的激光雷达，局部优化的扫描到地图的匹配是有效且稳健的（本论文使用的方法）。在不稳定的平台上，使用惯性测量单元（IMU）将激光扇（laser fan）投影到水平面上以估计重力的方向。

Pixel-accurate scan matching approaches, such as [1], further reduce local error accumulation. Although computationally more expensive, this approach is also useful for loop closure detection. Some methods focus on improving on the computational cost by matching on extracted features from the laser scans [4]. Other approaches for loop closure detection include histogram-based matching [6], feature detection in scan data, and using machine learning [7].
像素级精确的扫描匹配方法，例如[1]，进一步减少了局部累积误差。 虽然计算代价更昂贵，但这种方法对于回环检测也很有用。 一些方法侧重于通过提高匹配激光扫描中提取的特征来减少计算成本[4]。 用于回环检测的其他方法包括基于直方图的匹配[6]，扫描数据中的特征检测，以及使用机器学习的方法[7]。

Two common approaches for addressing the remaining local error accumulation are particle filter and graph-based SLAM [2],[8].
解决残留的局部累积误差的两种常用方法是采用粒子滤波器和基于图的SLAM [2]，[8]。

Particle filters must maintain a representation of the full system state in each particle. For grid-based SLAM, this quickly becomes resource intensive as maps become large; e.g. one of our test cases is 22,000 m2 collected over a 3km trajectory. Smaller dimensional feature representations, such as [9], which do not require a grid map for each particle, may be used to reduce resource requirements. When an up-todate grid map is required, [10] suggests computing submaps, which are updated only when necessary, such that the final map is the rasterization of all submaps.
粒子滤波器必须在每个粒子中保持完整系统状态的表示。 对于基于栅格的SLAM方法，地图变大则引起系统的状态计算快速消耗大量资源。
例如，我们的一个测试案例是在3公里的轨道上收集22,000平方米的数据计算。 可以使用较小维度的特征表示，例如[9]，其不需要每个粒子的栅格地图，以减少计算资源的需求。 当需要最新的栅格地图时，文献[10]建议计算子地图，仅在必要时更新，使得最终地图是所有子图的光栅化(rasterization)。

Graph-based approaches work over a collection of nodes representing poses and features. Edges in the graph are constraints generated from observations. Various optimization methods may be used to minimize the error introduced by all constraints, e.g. [11], [12]. Such a system for outdoor SLAM that uses a graph-based approach, local scan-to-scan matching, and matching of overlapping local maps based on histograms of submap features is described in [13].
基于图论的方法工作于节点表示位姿势和特征的集合上。 图中的边表示从观察结果中生成的约束(节点表示位姿和特征点)。 可以使用各种优化方法来最小化所有约束引入的误差，例如[11]、[12]。 文献[13]中讨论了一种用于室外SLAM的系统，它使用基于图论的方法、局部扫描到扫描的匹配、以及基于子图特征的直方图重叠局部图的匹配。

## System Overview
Google’s Cartographer provides a real-time solution for indoor mapping in the form of a sensor equipped backpack that generates 2D grid maps with a r=5cm resolution. The operator of the system can see the map being created while walking through a building. Laser scans are inserted into a submap at the best estimated position, which is assumed to be sufficiently accurate for short periods of time. Scan matching happens against a recent submap, so it only depends on recent scans, and the error of pose estimates in the world frame accumulates.
Google的Cartographer(制图师)提供了一种实时的室内地图解决方案，配置于后背式的一个传感器上，可生成r=5cm分辨率的2D网格地图。系统的操作员可以在穿越建筑物时看到创建的地图。激光的扫描帧被插入到经最佳位置估计的子图中，这是因为我们通常假定短时间内它的精确足够准确。 扫描匹配发生在最近的子图上，因此它仅与最近的扫描帧产生依赖，并在全局帧中估算位姿的累积误差。

To achieve good performance with modest hardware requirements, our SLAM approach does not employ a particle filter. To cope with the accumulation of error, we regularly run a pose optimization. When a submap is finished, that is no new scans will be inserted into it anymore, it takes part in scan matching for loop closure. All finished submaps and scans are automatically considered for loop closure. If they are close enough based on current pose estimates, a scan matcher tries to find the scan in the submap. If a sufficiently good match is found in a search window around the currently estimated pose, it is added as a loop closing constraint to the optimization problem. By completing the optimization every few seconds, the experience of an operator is that loops are closed immediately when a location is revisited. This leads to the soft real-time constraint that the loop closure scan matching has to happen quicker than new scans are added, otherwise it falls behind noticeably. We achieve this by using a branch-and-bound approach and several precomputed grids per finished submap.
为了在适度的硬件资源下获得良好的计算性能，我们的SLAM方法并不采用粒子滤波器。为了解决积累误差的问题，我们定时运行位姿优化模块。当一个子图完成后，即不再插入新的扫描帧时，子图将参与扫描匹配以闭合回环。所有已完成的子图和扫描帧都会自动考虑以闭合回环。如果它们与当前估计的位姿估计值距离足够近时，则扫描匹配器尝试在子图中搜索候选扫描帧。如果在当前估计的位姿周围的搜索窗中找到更好的匹配，则将其作为回环约束添加到优化问题中。每隔几秒完成优化过程，这样操作员的经验变成在某一位置被重新访问时立即闭合回环。这将导致软实时约束，即回环检测中的扫描匹配必须比添加新扫描帧更快发生，否则会明显落后。我们通过使用分支定界方法和对每个完成 子图上的几个预计算的栅格地图来实现这一点。

## Local 2D SLAM
Our system combines separate local and global approaches to 2D SLAM. Both approaches optimize the pose, $\xi=(\xi_x, \xi_y, \xi_\theta)$ consisting of $a(x, y)$ translation and a rotation $\xi_\theta$, of LIDAR observations, which are further referred to as `scans`. On an unstable platform, such as our backpack, an IMU is used to estimate the orientation of gravity for projecting scans from the horizontally mounted LIDAR into the 2D world. In our local approach, each consecutive scan is matched against a small chunk of the world, called a submap $M$, using a non-linear optimization that aligns the scan with the submap; this process is further referred to as `scan matching`. Scan matching accumulates error over time that is later removed by our global approach, which is described in Section V.
我们的系统将独立的局部方法和全局方法整合到2D SLAM中。 两种方法都优化了姿势，$\xi=(\xi_x, \xi_y, \xi_\theta)$，由LIDAR的观测所组成，分为平移量$a(x, y)$和旋转量$\xi_\theta$， 它们进一步统称为`扫描`。 在不稳定的平台上，例如我们的背包，IMU用于估计重力方向以将扫描从水平安装的LIDAR投影到2D世界。 在我们的局部方法中，每个连续扫描帧与整个构建的世界地图的一小块进行匹配（称为子图$ M $），使用非线性优化将扫描帧与子图对齐; 该过程进一步称为`扫描匹配`。 扫描匹配随着时间累积的误差会在之后被我们的全局方法删除，这部分内容将在第V节阐述。

## Closing Loops