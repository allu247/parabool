#!/usr/bin/env python
__author__ = 'Rasmus Tomsen'

import matplotlib.pyplot as plt
import csv
import numpy as np


def read_csv_file():
    satellite_times = []
    satellite_azimuths = []
    satellite_elevations = []
    antenna_azimuths = []
    antenna_elevations = []
    turning_times = []
    azimuth_displacements = []
    elevation_displacements = []
    azimuth_initial_velocities = []
    elevation_initial_velocities = []
    azimuth_final_velocities = []
    elevation_final_velocities = []
    azimuth_accelerations = []
    elevation_accelerations = []

    with open("../output/antenna_movement_data.csv", 'r') as csvfile:
        output = list(csv.reader(csvfile, delimiter=',', quotechar='|'))[1:]
        for row in output:
            satellite_times.append(row[0])
            satellite_azimuths.append(row[1])
            satellite_elevations.append(row[2])
            antenna_azimuths.append(row[3])
            antenna_elevations.append(row[4])
            turning_times.append(row[5])
            azimuth_displacements.append(row[6])
            elevation_displacements.append(row[7])
            azimuth_initial_velocities.append(row[8])
            elevation_initial_velocities.append(row[9])
            azimuth_final_velocities.append(row[10])
            elevation_final_velocities.append(row[11])
            azimuth_accelerations.append(row[12])
            elevation_accelerations.append(row[13])
            print(row)

    create_coordinate_plots(satellite_azimuths, antenna_azimuths, satellite_elevations, antenna_elevations)


def create_coordinate_plots(sat_az, ant_az, sat_el, ant_el):
    ant_az.pop(0)
    del sat_az[-1]
    ant_el.pop(0)
    del sat_el[-1]
    t = np.arange(0.0, len(sat_az), 1)

    plt.figure(1)
    plt.subplot(121)
    plt.plot(t, sat_az, 'ro', label='Satellite')
    plt.plot(t, ant_az, 'y--', label='Antenna', linewidth=2)
    plt.ylabel("Azimuth (rad)")
    plt.xlabel("Step")
    plt.title("Pass azimuths")
    plt.legend(loc='best')

    plt.subplot(122)
    plt.plot(t, sat_el, 'ro', label='Satellite')
    plt.plot(t, ant_el, 'y--', label='Antenna', linewidth=2)
    plt.ylabel("Elevation (rad)")
    plt.xlabel("Step")
    plt.title("Pass elevations")
    plt.legend(loc='best')
    plt.show()


if __name__ == '__main__':
    read_csv_file()
