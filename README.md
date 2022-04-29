<head>
    <script src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML" type="text/javascript"></script>
    <script type="text/x-mathjax-config">
        MathJax.Hub.Config({
            tex2jax: {
            skipTags: ['script', 'noscript', 'style', 'textarea', 'pre'],
            inlineMath: [['$','$']]
            }
        });
    </script>
</head>
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

## For details
The output of MPC is linear velocity **'v'** and angular velocity **'w'**. We provide two mpc model, here is the details.

### control velocity:
* **Continuous Model**:
$$
\begin{cases}
\dot{x}=v\cdot \cos\theta\\
\dot{y}=v\cdot \sin\theta\\
\dot{\theta}=\omega\\
\end{cases}
$$
* **Discrete Model** ( linearize the model at one point ):
Let model be: $X_{k+1}=A(X_k)X_k+B(X_k)U_k+C(X_k)$
$$
\begin{cases}
x_{k+1}=x_k-v_k\cdot \sin\theta_k\cdot(\theta-\theta_k)\cdot\Delta t+\cos\theta_k\cdot(v-v_k)\cdot \Delta t+v_k\cdot\cos\theta_k\cdot \Delta t\\
y_{k+1}=y_k+v_k\cdot \cos\theta_k\cdot(\theta-\theta_k)\cdot\Delta t+\sin\theta_k\cdot(v-v_k)\cdot \Delta t+v_k\cdot\sin\theta_k\cdot \Delta t\\
\theta_{k+1}=\theta_k+(\omega-\omega_k)\cdot \Delta t+\omega_k\cdot\Delta t
\end{cases}
$$Note that we can get $v_k$ and $\omega_k$ by Odometry.
* **Optimization Problem**:
Let $x_k:=\{x_k,y_k,\theta_k\},u_k:=\{v_k,\omega_k\}$ then we have:
$$
\begin{align}
\min_{u_k,\forall k}{}
& J=\sum_{k=1}^{N}||x_k-x_k^{ref}||_{Q_x}^2+\sum_{k=0}^{N-1}(||u_k-u_k^{ref}||_{Q_u}^2+||u_k||_R^2)+\sum_{k=1}^{N-1}||u_k-u_{k-1}||_{R_d}^2\\
\text{s.t.}\
& x_{k+1}=A(x_k)\cdot x_k+B(x_k)\cdot u_k+ C(x_k), \quad \ \ k=0,1,...,N-1\\
& ||u_k||\le u_{max},\quad \quad \quad \quad \quad \quad \quad \quad \quad \quad  \quad \quad \quad k=0,1,...,N-1\\
& ||u_{k}-u_{k-1}||\le d_{max},\quad \quad \quad \quad \quad \quad \quad \quad \quad \quad k=1,2,...,N-1\\
\end{align}
$$
### control acceleration:
* **Continuous Model**:
$$
\begin{cases}
\dot{x}=v\cdot \cos\theta\\
\dot{y}=v\cdot \sin\theta\\
\dot{v}=a\\
\dot{\theta}=\omega\\
\end{cases}
$$
* **Discrete Model** ( linearize the model at one point ):
Let model be: $X_{k+1}=A(X_k)X_k+B(X_k)U_k+C(X_k)$
$$
\begin{cases}
x_{k+1}=x_k-v_k\cdot \sin\theta_k\cdot(\theta-\theta_k)\cdot\Delta t+\cos\theta_k\cdot(v-v_k)\cdot \Delta t+v_k\cdot\cos\theta_k\cdot \Delta t\\
y_{k+1}=y_k+v_k\cdot \cos\theta_k\cdot(\theta-\theta_k)\cdot\Delta t+\sin\theta_k\cdot(v-v_k)\cdot \Delta t+v_k\cdot\sin\theta_k\cdot \Delta t\\
v_{k+1} = v_k+(a-a_k)\cdot \Delta t+a_k\cdot\Delta t\\
\theta_{k+1}=\theta_k+(\omega-\omega_k)\cdot \Delta t+\omega_k\cdot\Delta t
\end{cases}
$$Note that we can get $v_k$ and $\omega_k$ by Odometry.
* **Optimization Problem**:
Let $x_k:=\{x_k,y_k,\theta_k\},u_k:=\{v_k,\omega_k\}$ then we have:
$$
\begin{align}
\min_{u_k,\forall k}{}
& J=\sum_{k=1}^{N}||x_k-x_k^{ref}||_{Q_x}^2+\sum_{k=0}^{N-1}(||u_k-u_k^{ref}||_{Q_u}^2+||u_k||_R^2)+\sum_{k=1}^{N-1}||u_k-u_{k-1}||_{R_d}^2\\
\text{s.t.}\
& x_{k+1}=A(x_k)\cdot x_k+B(x_k)\cdot u_k+ C(x_k), \quad \ \ k=0,1,...,N-1\\
& ||u_k||\le u_{max},\quad \quad \quad \quad \quad \quad \quad \quad \quad \quad  \quad \quad \quad k=0,1,...,N-1\\
& ||u_{k}-u_{k-1}||\le d_{max},\quad \quad \quad \quad \quad \quad \quad \quad \quad \quad k=1,2,...,N-1\\
\end{align}
$$
