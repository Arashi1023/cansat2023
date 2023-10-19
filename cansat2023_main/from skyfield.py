from skyfield.api import Topos, load
import matplotlib.pyplot as plt
import matplotlib.animation as animation

stations_url = 'https://celestrak.org/NORAD/elements/gp.php?GROUP=starlink&FORMAT=tle'
satellites = load.tle_file(stations_url)

ts = load.timescale()
t = ts.now()

geocentric_positions = [satellite.at(t).position.km for satellite in satellites]

fig = plt.figure()

def animate(i):
    fig.clear()
    t = ts.utc(2023, 10, 4, i, 0, 0)
    geocentric_positions = [satellite.at(t).position.km for satellite in satellites]
    for position in geocentric_positions:
        x, y, z = position
        plt.scatter(x, y)
    plt.xlim(-10000, 10000)
    plt.ylim(-10000, 10000)

ani = animation.FuncAnimation(fig, animate, frames=24*60, interval=200)
plt.show()
