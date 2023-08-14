import argparse

import yaml
import numpy as np
import matplotlib.pyplot as plt

from mpc import LTIMPC
import model


def parse_argument():

    parser = argparse.ArgumentParser()
    parser.add_argument('--steps', type=int, default=120,
                        help='the number of simulation step')
    parser.add_argument('--cfg', default='cfg/cfg.yaml',
                        help='configration file')
    parser.add_argument('--output', default='result')

    return parser.parse_args()


def extract_cost(cfg):
    Q = np.array(cfg['cost_function']['Q'])
    R = np.array(cfg['cost_function']['R'])

    return Q, R


def extract_constraint(cfg):
    F = np.array(cfg['constraint']['F'])
    f = np.array(cfg['constraint']['f'])
    G = np.array(cfg['constraint']['G'])
    g = np.array(cfg['constraint']['g'])

    return F, f, G, g


def main():

    args = parse_argument()

    with open(args.cfg) as f:
        cfg = yaml.safe_load(f)

    Q, R = extract_cost(cfg)
    F, f, G, g = extract_constraint(cfg)

    mpc = LTIMPC(cfg['horizon'],
                 Q, R,
                 model.A, model.B,
                 F, f,
                 G, g)

    x = np.array(cfg['initial_state'])

    u_hist = []
    x_hist = [x]
    m = model.B.shape[1]
    for i in range(1, args.steps+1):

        u_aug, status = mpc.solve(x)
        if not status:
            print(f'{i}: optimal solution is not found.')

        u = u_aug[0:m]
        x = model.A @ x + model.B @ u

        u_hist.append(u[0, 0])
        x_hist.append(x)

    x_hist = np.array(x_hist)
    u_hist = np.array(u_hist)

    plt.figure(1, (12, 6))
    plt.subplots_adjust(left=0.05, right=0.95)
    plt.subplot(1, 2, 1)
    plt.title('State')
    plt.scatter(x_hist[:, 0], x_hist[:, 1], facecolors='none', edgecolors='b')
    plt.xlabel('Position')
    plt.ylabel('Velocity')
    plt.grid(True)

    plt.subplot(1, 2, 2)
    plt.title('Control Input')
    plt.plot(np.arange(len(u_hist)), u_hist, 'o-',
             markerfacecolor='none', color='darkblue',
             markersize=5, linewidth=1)
    plt.xlabel('Step')
    plt.grid(True)

    plt.savefig(args.output+'.png')
    # plt.show()


if __name__ == '__main__':
    main()
