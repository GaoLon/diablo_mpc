# diablo_mpc

## Quick Start

A **MPC** for **diablo** configuration(see as differential car).
If you want to use the code, you should install `OSQP`, `osqp-eigen` and`serial` first, and can following steps:

```
cd test_mpc
catkin_make
source devel/setup.bash
roslaunch mpc test_mpc.launch
```

## Details

The output of MPC is linear velocity **'v'** and angular velocity **'w'**. We provide two mpc model, here is the details.

### Control Velocity

* **Continuous Model**:
$$
\begin{cases}
\dot{x}=v\cdot \cos\theta\\
\dot{y}=v\cdot \sin\theta\\
\dot{\theta}=\omega\\
\end{cases}
$$
* **Discrete Model** ( linearize the model at one point $\hat{X},\hat U$):
Let state and output be $X_k=[x_k,y_k,\theta_k]^T,U_k=[v_k,\omega_k]^T$
Let model be: $X_{k+1}=A(\hat X,\hat U)X_k+B(\hat X,\hat U)U_k+C(\hat X,\hat U)$
$$
\begin{cases}
x_{k+1}=x_k-\hat v\cdot \sin\hat\theta\cdot(\theta_k-\hat\theta)\cdot\Delta t+\cos\hat\theta\cdot(v_k-\hat v)\cdot \Delta t+\hat v\cdot\cos\hat\theta\cdot \Delta t\\
y_{k+1}=y_k+\hat v\cdot \cos\hat\theta\cdot(\theta_k-\hat\theta)\cdot\Delta t+\sin\hat\theta\cdot(v_k-\hat v)\cdot \Delta t+\hat v\cdot\sin\hat\theta\cdot \Delta t\\
\theta_{k+1}=\theta_k+(\omega_k-\hat\omega)\cdot \Delta t+\hat\omega\cdot\Delta t
\end{cases}
$$Note that we can get $\hat X$ and $\hat U$ by Odometry.
* **Optimization Problem**:
We have:
$$
\begin{align}
\min_{U_k,\forall k}{}
& J=\sum_{k=1}^{N}||X_k-X_k^{ref}||_{Q_x}^2+\sum_{k=0}^{N-1}(||U_k-U_k^{ref}||_{Q_u}^2+||U_k||_R^2)+\sum_{k=1}^{N-1}||U_k-U_{k-1}||_{R_d}^2\\
\text{s.t.}\
& X_{k+1}=A_k X_k+B_kU_k+ C_k, \quad \quad \quad \quad \quad \quad \quad k=0,1,...,N-1\\
& ||U_k||\le U_{max},\quad \quad \quad \quad \quad \quad \quad \quad \quad \quad  \quad \quad \quad \ k=0,1,...,N-1\\
& ||X_k||\le X_{max},\quad \quad \quad \quad \quad \quad \quad \quad \quad \quad  \quad \quad \quad k=1,2,...,N-1\\
& ||U_{k}-U_{k-1}||\le U_{dmax},\quad \quad \quad \quad \quad \quad \quad \quad \quad \ \ \ k=1,2,...,N-1\\
\end{align}
$$

### Control Acceleration

* **Continuous Model**:
$$
\begin{cases}
\dot{x}=v\cdot \cos\theta\\
\dot{y}=v\cdot \sin\theta\\
\dot{v}=a\\
\dot{\theta}=\omega\\
\end{cases}
$$
* **Discrete Model** ( linearize the model at one point $\hat{X},\hat U$):
Let state and output be $X_k=[x_k,y_k,v_k,\theta_k]^T,U_k=[a_k,\omega_k]^T$
Let model be: $X_{k+1}=A(\hat X,\hat U)X_k+B(\hat X,\hat U)U_k+C(\hat X,\hat U)$
$$
\begin{cases}
x_{k+1}=x_k-\hat v\cdot \sin\hat\theta\cdot(\theta_k-\hat\theta)\cdot\Delta t+\cos\hat\theta\cdot(v_k-\hat v)\cdot \Delta t+\hat v\cdot\cos\hat\theta\cdot \Delta t\\
y_{k+1}=y_k+\hat v\cdot \cos\hat\theta\cdot(\theta_k-\hat\theta)\cdot\Delta t+\sin\hat\theta\cdot(v_k-\hat v)\cdot \Delta t+\hat v\cdot\sin\hat\theta\cdot \Delta t\\
v_{k+1}=v_k+(a_k-\hat a)\cdot \Delta t+\hat a\cdot \Delta t\\
\theta_{k+1}=\theta_k+(\omega_k-\hat\omega)\cdot \Delta t+\hat\omega\cdot\Delta t
\end{cases}
$$Note that we can get $\hat X$ and $\hat U$ by Odometry.
* **Optimization Problem**:
We have the same fomula as control velocity but adjust some parameters.

### Choosing Parameters

* **Control Velocity:**
$$
\begin{cases}
Q_x=diag(10, 10, 0.5)\\
Q_u=diag(2.5,0)\\
R=diag(0.01,0.01)\\
R_d=diag(0.01,1.0)\\
U_{max}=[1.5,2.4]^T\\
X_{max}=[\infty,\infty,\infty]^T\\
U_{dmax}=[0.5,1.0]^T\\
\end{cases}
$$
* **Control Acceleration:**
$$
\begin{cases}
Q_x=diag(10, 10,2.5, 0.5)\\
Q_u=diag(0,0)\\
R=diag(0.01,0.01)\\
R_d=diag(0.01,1.0)\\
U_{max}=[0.5,2.4]^T\\
X_{max}=[\infty,\infty,1.5,\infty]^T\\
U_{dmax}=[\infty,1.0]^T\\
\end{cases}
$$

### Notation

* Models are just for forward tracking, but we can track backward by inversing the model. Besides, the algorithm will choose a better way to track the path.
* When prediction, we don't choose just one point to linearize the model, that isn't precise enough.
* MPC will run iteratively until converge or at max loops.
* 100Hz with 100 predict steps is recommanded for consideration of effect and efficiency.
