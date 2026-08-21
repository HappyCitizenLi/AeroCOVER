# SOFT-VoFOD 公式与代码映射

状态日期：2026-08-21

## 1. Background evidence

对 voxel `v`：

$$
P_F(v)=\frac{F_v}{F_v+O_v+\epsilon},\qquad
P_B(v)=\frac{O_v}{F_v+O_v+\epsilon}
$$

$$
C(v)=1-\exp\left(-\frac{F_v+O_v}{N_0}\right)
$$

`F_v` 是路径长度归一化后的 free evidence，`O_v` 是独立 endpoint group 的 background
evidence，`N_0` 对应 `map/evidence_scale`。实现位于
`soft_vofod::BackgroundMap::query/updateState/carveFreeRay`。

event score：

$$
a_j=C(v_j)P_F(v_j)\min\left(1,\frac{d_{bg,j}}{d_0}\right)
$$

其中 `d_bg` 是配置搜索半径内最近 stable-background voxel center 距离，找不到时取搜索半径；
`d_0` 对应 `event/distance_scale_m`。实现位于 `SoftVofodCore::processBatch` 的旧地图查询段。

## 2. Trajectory birth

事件模型：

$$
z_n=p_0+(t_n-t_0)v+\epsilon_n
$$

每个 hypothesis 最多保留每个 independent group 的一个 inlier。refit 使用权重
`w_n=max(0.05,a_n)`：

$$
\bar t=\frac{\sum_n w_nt_n}{\sum_nw_n},\qquad
\bar z=\frac{\sum_n w_nz_n}{\sum_nw_n}
$$

$$
\hat v=\frac{\sum_n w_n(t_n-\bar t)(z_n-\bar z)}
{\sum_n w_n(t_n-\bar t)^2},\qquad
\hat p(t)=\bar z+\hat v(t-\bar t)
$$

$$
e_{RMS}=\sqrt{\frac{\sum_nw_n\|z_n-\hat p(t_n)\|^2}{\sum_nw_n}}
$$

实现位于 `SoftVofodCore::bestBirthCandidate/createBirth`。`measurement_variance_m2 +
shape_sigma_m²` 是当前没有 observer covariance 时的统一 fallback；接口结构允许后续在生成
event/measurement 时加入 pose covariance，但当前没有假装使用 truth covariance。

## 3. CV-KF

$$
x=[p^T,v^T]^T,\qquad
F(\Delta t)=\begin{bmatrix}I&\Delta tI\\0&I\end{bmatrix}
$$

$$
Q(\Delta t)=\sigma_a^2
\begin{bmatrix}
\frac{\Delta t^4}{4}I&\frac{\Delta t^3}{2}I\\
\frac{\Delta t^3}{2}I&\Delta t^2I
\end{bmatrix}
$$

$$
z=Hx+\nu,\qquad H=[I,0],\qquad
R=(\sigma_{sensor}^2+\sigma_{shape}^2)I
$$

预测和 Joseph-form correction 位于 `SoftVofodCore::transition/processNoise/predictTracks` 与
`processBatch`。所有 covariance 参数单位均为 variance，不是 standard deviation。

## 4. Association

$$
d^2_{ij}=(z_j-H\hat x_i)^TS_i^{-1}(z_j-H\hat x_i)
$$

$$
C_{ij}=d^2_{ij}+\lambda_a(1-a_j)
$$

超出 `association_gate_d2` 的项为禁止边；dummy assignment cost 高于所有合法 gate cost。
`SoftVofodCore::hungarian` 返回每个 track 至多一个 measurement，packet aggregation 再保证每个
局部 return 只有一个 owner。

## 5. Scan opportunity

3D position covariance 分解为主轴，使用七个等权 sigma points：

$$
\chi=\{\mu,\mu\pm s\sqrt{\lambda_1}e_1,
\mu\pm s\sqrt{\lambda_2}e_2,\mu\pm s\sqrt{\lambda_3}e_3\}
$$

$$
\eta_{ij}=\frac{1}{7}\sum_{n=1}^{7}
\mathbf 1[\text{ray}_j\cap\text{Sphere}(\chi_n,R_{target})]
$$

若真实 return 或另一 confirmed track 位于该 sigma sphere near range 前方，相应 indicator 为零。

$$
P_{D,i}=\min\left(P_{D,max},
1-\prod_j(1-p_{ret}\eta_{ij})\right)
$$

乘积使用 `log1p/expm1` 计算。实现位于
`SoftVofodCore::raySphereNearRange/opportunity/detectionProbability`。

## 6. Existence

有 opportunity 但未匹配：

$$
r^+=\frac{r^-(1-P_D)}{1-r^-P_D}
$$

所以 `P_D=0` 时严格有 `r^+=r^-`。

命中 likelihood：

$$
\ell(z)=\frac{\exp(-\frac12d_M^2)}
{\sqrt{(2\pi)^3\det S}}
$$

$$
r^+=\frac{r^-P_D\ell}
{r^-P_D\ell+(1-r^-)\kappa}
$$

命中公式在 log domain 求值，见 `SoftVofodCore::missedExistence/hitExistence`。

## 7. Ray truncation

对期望 carving 长度 `L` 和所有 target/quarantine support 的近交点 `r_near,k`：

$$
L^*=\min\left(L,\min_k\max(0,r_{near,k}-g_{target})\right)
$$

VALID_RETURN 的原始 `L=r-g_endpoint`；NO_RETURN 的原始
`L=max_no_return_free_range_m`。实现位于
`SoftVofodCore::supports/truncateBeforeSupport` 和 `BackgroundMap::carveFreeRay`。

