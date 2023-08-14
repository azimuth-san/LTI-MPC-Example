## Optimization Problem 

$$
\underset{u_{k+i|k}: ~i\in \mathbb{N}_{0:N-1}}{\rm{minimize}} \quad \sum_{i=0}^{N-1} \{ \hat{x}^T_{k+i|k} Q \hat{x}_{k+i|k} + \hat{u}^T_{k+i|k} R \hat{u}_{k+i|k} \} + \hat{x}^T_{k+N|k} Q_f \hat{x}_{k+N|k} \\[5mm]
{\rm{s.t.}} \quad x_{k+i+1|k} = A x_{k+i|k} + B u_{k+i|k} , \quad i\in \mathbb{N}_{0:N-1}\\[2mm]
\quad u_{k+i|k} \in \{u \in \mathbb{R}^m ~| Gu \leq g \} , \quad i\in \mathbb{N}_{0:N-1} \\[2mm]
\quad x_{k+i|k} \in \{x \in \mathbb{R}^n ~| Fx \leq f \} , \quad i\in \mathbb{N}_{0:N-1} \\[2mm]
x_{k+N|k} \in \{x \in \mathbb{R}^n ~| F_f x \leq f_f \} \\
$$

<br>

## Example1

$$
x_{t+1} = 
\begin{bmatrix}
1 & T \\
0 & 1
\end{bmatrix}
x_{t} + 
\begin{bmatrix}
0 \\
T 
\end{bmatrix}
u_t, \quad T = 0.05 \\[5mm]
$$

$$
F x_t \leq f, \quad F = 
\begin{bmatrix}
1 & 0  \\
-1 & 0  \\
0 & 1  \\
0 & -1  \\
\end{bmatrix},
\quad f = 
\begin{bmatrix}
10  \\
10  \\
10  \\
10  \\
\end{bmatrix} \\[2mm]
\quad G u_t \leq g , \quad G = 
\begin{bmatrix}
1  \\
-1 \\
\end{bmatrix},
\quad g = 
\begin{bmatrix}
15  \\
15 \\
\end{bmatrix} \\[5mm]
$$

$$
x_0 = [5, ~10]^T
$$

