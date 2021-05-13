from boids import Boids, tick, height, width
import argparse
import matplotlib.pyplot as plt
import matplotlib.animation as animation

if __name__ == '__main__':
    print("Starting boids.")

    parser = argparse.ArgumentParser(description="Implementing Craig Reynold's Boids.")

    # adding arguments
    parser.add_argument('--num-boids', dest='N', required=False)
    args = parser.parse_args()

    N = 100
    if args.N:
        N = int(args.N)

    boids = Boids(N)

    fig = plt.figure()
    ax = plt.axes(xlim=(0, width), ylim=(0,height))
    # plt.plot([300,20],[300,50])

    body, = ax.plot([],[], markersize=10, c='k', marker='o', ls='None')
    head, = ax.plot([],[], markersize=4, c='r', marker='o', ls='None')
    ani = animation.FuncAnimation(fig, tick, fargs=(body, head, boids), interval=50)

    # cid = fig.canvas.mpl_connect('button_press_event', boids.button_press)

    plt.show()
