# VIO 第五讲作业	

## 基础题

### 1. 完成 problem.cc 当中的部分代码

- 完整的代码直接见压缩包里面的 `problem.cpp` 

  <img src="/home/kyletian/vio/ch5/images/Screenshot from 2022-07-23 21-36-02.png" style="zoom:80%;" />

  <img src="/home/kyletian/Pictures/Screenshot from 2022-07-23 21-37-47.png" style="zoom:80%;" />

  ![](/home/kyletian/Pictures/Screenshot from 2022-07-23 21-37-56.png)

  ![](/home/kyletian/Pictures/Screenshot from 2022-07-23 21-38-05.png)

  

  

### 2.  完成滑动窗口算法测试函数

- 完整代码还是见 `problem.cpp`

  ![](/home/kyletian/Pictures/Screenshot from 2022-07-23 21-38-29.png)

  ![](/home/kyletian/Pictures/Screenshot from 2022-07-23 21-38-36.png)

这两个部分都完成之后，得到的运行结果为：

<img src="/home/kyletian/vio/ch5/images/origin_result.png" style="zoom:67%;" />

从结果上来看并没有什么问题，优化值和真实值之间的差别非常小。Test Marginalization 测试里面那个矩阵也正确地进行了移动。

## 附加题

### 3. 论文总结

#### **<u>三种在基于优化的视觉惯性状态估计中 处理规范自由度的不同方法：</u>**

- 将不可观察的状态固定为一些给定的值（作为参数）gauge fixation
- 设置这些状态的先验  gauge prior
- 让状态在优化过程中自由演化  free gauge

<!--For a VI system, global position and yaw are not observable. 称他们为 gauge freedom。这个也就是信息矩阵 H 的自由度。它们通常作为实现细节呈现，因此没有得到很好的研究和理解。-->

<!--虽然所有这些方法在估计误差方面都具有相似的性能，但自由规范方法稍快一些，因为收敛所需的迭代次数较少。-->

<!--在自由规范方法中，优化产生的协方差与任何特定的参考框架无关（与规范固定方法的参考框架相反），这使得难以以有意义的方式解释不确定性。然而，在这项工作中，我们进一步表明，通过应用协方差变换，自由规范协方差实际上与其他方法密切相关（见图 1）。-->

<!--gauge $\mathcal{C}$ : 添加一个额外的约束，得到一个唯一解。在 VI 中，这是通过为 3D 重建指定参考坐标系来实现的-->

问题：即使我们对状态（参数向量）θ 的所有元素使用最小参数化，驱动参数更新的 (1) 的 Hessian 矩阵由于不可观察的自由度而是奇异的。更具体地说，它的秩不足为 4，对应于 (4) 中的 4-DoF。

- Gauge fixation: 在没有不可观察状态的较小参数空间中进行优化，因此 Hessian 是可逆的。这实质上对解决方案施加了硬约束
- Gauge Prior: 用额外的惩罚（产生可逆的 Hessian）来增加目标函数，以支持解决方案以软方式满足某些约束
- Free Gauge: 可以使用奇异 Hessian 的伪逆来隐式地为唯一解决方案提供额外的约束（具有最小范数的参数更新）

**gauge fixation 和 gauge prior：**

需要固定相机姿势的 1-DoF yaw 旋转角度。通常使用第一个相机的 pose 来对 yaw 进行固定。使用左乘增量的方法：
$$
R_{0} = \mathrm{exp}(\Delta \phi_{0}) R_{0}^{0}
$$
通过查网上其他帖子，我知道这样做是能够保证 $R_{0}^{0}$ 和最后的 $R_{0}$ 只有一次相乘运算，也就是只有一次旋转。和另外一个公式相比，即使每次旋转的时候都强制 z 方向上的分量为0，只要 x轴 和 y轴 不与最初的 z轴 正交，那么旋转矢量在最初的 z轴 方向还是会存在不为 0 的分量。

如果是 gauge fixation，则直接：

<img src="/home/kyletian/vio/ch5/images/OneDrive-2022-07-23/image-20220723151205918.png" alt="image-20220723151205918" style="zoom: 50%;" />

如果是 gauge prior (增加先验)，则是往 cost function 里面增加一个惩罚量：

<img src="/home/kyletian/vio/c![prior_constraint](/home/kyletian/vio/ch5/images/prior_constraint.png)h5/images/OneDrive-2022-07-23/image-20220723151359256.png" alt="image-20220723151359256" style="zoom:50%;" />

<img src="/home/kyletian/vio/ch5/images/OneDrive-2022-07-23/image-20220723152332485.png" alt="image-20220723152332485" style="zoom: 50%;" />

正如论文里面所说的，如果 wp = 0，那么就等价于 free gauge，如果 wp 无穷大，则等价于 gauge fixation。

然后作者对于不同的先验权重进行了实验。虽然解决方案的精度对于不同的先验权重没有显着变化（图 4），但在规范先验方法中需要正确选择先验权重以保持较小的计算成本（图 5）。最好不要选择极大的权重，因为它们有时会使优化变得不稳定。

#### 三种方法的实验结果：

比较了三种方法在模拟轨迹（正弦、圆弧和rec）和3D点（平面和随机）的六种组合上的性能，得到下面的结论：

- 三种方法的表现几乎相同
- 如果采用 gauge prior 方法，则需要选择适当的先验权重以避免增加计算成本
- 在适当的权重下，gauge prior 方法具有与 gauge fixation 方法几乎相同的性能（准确性和计算成本）
- 自由规范方法比其他方法稍快，因为它需要更少的迭代来收敛

**自由规范方法具有通用的额外优势，即不特定于 VI，因此它不需要对旋转参数化进行任何特殊处理。**

### 4. 比较为 prior 设定不同权重时，BA 求解收敛精度和速度

- 这个题我一开始卡在如何手动给信息矩阵添加 prior 约束以及对应的权重上面。后来实在卡得太久了，就查了一下网上，然后发现 `edge_prior.cpp/edge_prior.h` 就是用来做这个的。然后后来看前面的代码，终于是把逻辑稍微捋顺了一点，但是我也不知道自己想的对不对

- 代码截图

  ![](/home/kyletian/vio/ch5/images/prior_constraint.png)

- 结果截图（部分）

  - 权重设置为 $10^{15}$

  <img src="/home/kyletian/vio/ch5/images/10e15.png" style="zoom: 67%;" />

  - 权重设置为 $10^{8}$

  <img src="/home/kyletian/vio/ch5/images/10e8.png" style="zoom: 67%;" />

  - 权重设置为 $10^{4}$

  <img src="/home/kyletian/vio/ch5/images/10e4.png" style="zoom:67%;" />

- 然后如果和 固定前两帧的位置的结果（`TestMonoBA.cpp`大概90行那里取消注释）相比较：

  <img src="/home/kyletian/vio/ch5/images/fix_first.png" style="zoom:67%;" />

- 结论：当先验的权重设置到很大的时候，经过优化之后的结果和真实值能够是一模一样的。但当权重设置的没那么大的时候，结果会和真实值有一点区别。当然这种区别非常小。在前面第一问当中，可以看出在初始帧当中优化值和真实值之间的差距还是有的，比这一问当中所采用的方法要大一些。

  至于速度，虽然在论文里提到采取合适的权重能够使得速度尽可能的快，但是在我自己的试验当中这种规律没有体现出来。problemSolveCost 和 HessianCost 都非常地不稳定，我也不确定是什么原因导致的。在我的机器上应该也需要运行很多次才能体现出来这种差距，但是我没有那么多时间去进行实验了。