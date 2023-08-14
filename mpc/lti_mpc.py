import numpy as np
import scipy.linalg as la
import cvxopt
from cvxopt import matrix


class LTIMPC:
    """
    A model precdicitive controller class for linear time invariant systems.

    Reference
    ---------
    [1] Michael Fink.
        Implementation of Linear Model Predictive Control - Tutorial.
        https://arxiv.org/abs/2109.11986v1

    [2] Francesco Borrelli, Alberto Bemporad, and Manfred Morari.
        Predictive Control for Linear and Hybrid Systems, Cambridge University Press, 2017.
    """

    def __init__(self, N, Q, R, A, B, F, f, G, g):
        """ Initialize.

        Parameters
        ----------
        N : int
            prediction horizon.
        Q, R : np.ndarray
               matrices of a cost function.
               x[t]' * Q * x[t] + u[t]' * R * u[t] + x[N] * Qf * x[N]
        A, B : np.ndarray
               matrices of a state space model for LTI systems.
               x[t+1] = A * x[t] + B * u[t]
        F, f : np.ndarray
               matrices of state constraints.
               F * x[t] <= f
        G, g : np.ndarray
               matrices of input constraints.
               G * u[t] <= g
        """
        # terminal cost matrix
        Qf = la.solve_discrete_are(A, B, Q, R)
        # print(Qf)

        A, B = self.augment_AB(N, A, B)
        F, f = self.augment_Ff(N, F, f)
        G, g = self.augment_Gg(N, G, g)

        Q = self.augment_Q(N, Q, Qf)
        R = self.augment_R(N, R)

        self.FG = np.vstack((F @ B, G))
        self.fg = np.vstack((f, g))
        FA = F @ A
        zero = np.zeros((self.fg.shape[0]-FA.shape[0], FA.shape[1]))
        self.FA = np.vstack((FA, zero))

        self.H = B.T @ Q @ B + R
        # self.q_pre = B.T @ A  # equation (44b) of the reference[1]
        self.q_pre = B.T @ Q @ A  # equation (8.8) of the  reference[2]

        cvxopt.solvers.options['show_progress'] = False

    @classmethod
    def augment_AB(cls, N, A, B):
        """ Augment A and B matrix.

        Parameters
        ----------
        N : int
            prediction horizon.
        A, B : np.ndarray
               matrices of a state space model for LIT systems.
               x[t+1] = A * x[t] + B * u[t]
        Returns
        --------
        A_aug, B_aug : np.ndarray
            A_aug = | I |  B_aug = | 0           0         ..  0 |
                    | A |          | B           0         ..  0 |
                    |A^2|          | A*B         B         ..  0 |
                    | : |          | :           :         ..  : |
                    |A^N|          |A^(N-1)*B  A^(N-2)*B   ..  B |
        """

        n, m = A.shape[0], B.shape[1]

        A_aug = np.zeros((n*(N+1), A.shape[1]))
        V = np.eye(n)
        for i in range(N+1):
            A_aug[i*n:(i+1)*n] = V
            V = V @ A

        B_base = np.zeros((n*(N+1), m))
        V = B
        for i in range(1, N+1):
            B_base[i*n:(i+1)*n] = V
            V = A @ V

        B_aug = np.zeros((n*(N+1), m*N))
        for j in range(N):
            B_aug[:, j*m:(j+1)*m] = B_base
            # slide
            V = B_base[:-n]
            B_base[n:] = V

        return A_aug, B_aug

    def augment_Ff(self, N, F, f, Ff=None, ff=None):
        """ Augment F and f matrix.

        Parameters
        ----------
        N : int
            prediction horizon.
        F, f : np.ndarray
                matrices of state constraints.
                F * x[t] <= f
        Ff, ff : np.ndarray
                 matrices of terminal state constraints.
                 Ff * x[N] <= ff
        Returns
        --------
        F_aug, f_aug : np.ndarray
            F_aug = | F  0 .. 0  0 |  f_aug = | f |
                    | 0  F .. 0  0 |          | f |
                    | :  : .. :  : |          | f |
                    | :  : .. F  0 |          | f |
                    | 0  0 .. 0  Ff|          | ff|
        """

        F_aug = np.kron(np.eye(N+1), F)
        f_aug = np.kron(np.ones((N+1, 1)), f)

        if Ff is not None:
            n = F.shape[1]
            F_aug[-n:, -n:] = Ff
            f_aug[-n:] = ff

        return F_aug, f_aug

    def augment_Gg(self, N, G, g):
        """ Augment G and g matrix.

        Parameters
        ----------
        N : int
            prediction horizon.
        G, g : np.ndarray
               matrices of input constraints.
               G*u <= g

        Returns
        --------
        G_aug : np.ndarray
            G_aug = | G  0 .. 0 |  g_aug = | g |
                    | 0  G .. 0 |          | g |
                    | :  : .. : |          | g |
                    | 0  0 .. Gf|          | g |
        """

        G_aug = np.kron(np.eye(N), G)
        g_aug = np.kron(np.ones((N, 1)), g)

        return G_aug, g_aug

    def augment_Q(self, N, Q, Qf):
        """ Augment Q matrix.

        Parameters
        ----------
        N : int
            prediction horizon.
        Q : np.ndarray
            a matrix of a stage cost.
        Qf : np.ndarray
             a matrix of a terminal cost.
             x[t]' * Q * x[t] + u[t]' * R * u[t] + x[N] * Qf * x[N]

        Returns
        --------
        Q_aug : np.array
                | Q  0 .. 0 |
                | 0  Q .. 0 |
                | :  : .. : |
                | 0  0 .. Qf|
        """

        n = Qf.shape[0]

        Q_aug = np.kron(np.eye(N+1), Q)
        Q_aug[-n:, -n:] = Qf

        return Q_aug

    def augment_R(self, N, R):
        """Augment R matrix.

        Parameters
        ----------
        N : int
            prediction horizon.
        R : np.ndarray
            a matrix of a input's cost.
            x[t]' * Q * x[t] + u[t]' * R * u [t]+ x[N] * Qf * x[N]

        Returns
        -------
        R_aug : np.ndarray
                | R  0 .. 0 |
                | 0  R .. 0 |
                | :  : .. : |
                | 0  0 .. R |
        """

        R_aug = np.kron(np.eye(N), R)
        return R_aug

    def solve(self, x):

        H = matrix(self.H)
        q = matrix(self.q_pre @ x)

        G = matrix(self.FG)
        g = matrix(self.fg) - matrix(self.FA @ x)

        sol = cvxopt.solvers.qp(H, q, G, g)
        u = np.array(sol['x'])

        status = True if sol['status'] == 'optimal' else False

        return u, status
