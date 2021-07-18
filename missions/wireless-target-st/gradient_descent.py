"""
This file contains the code for the hotspot localization algorithm.
"""

import math
import random

import numpy as np
import utm


def GPS2UTM(data):
    x = np.zeros(len(data))
    y = np.zeros(len(data))
    z = np.zeros(len(data))
    for i in range(0, len(data)):
        x[i] = np.array(utm.from_latlon(data[i][0], data[i][1])[0])
        y[i] = np.array(utm.from_latlon(data[i][0], data[i][1])[1])
        z[i] = np.array(data[i][2])
        zone_number = utm.from_latlon(data[0][0], data[0][1])[2]
        zone_letter = utm.from_latlon(data[0][0], data[0][1])[3]
    return x, y, z, zone_number, zone_letter


def GPS2UTM_raw(lat, long, alt):
    x = utm.from_latlon(lat, long)[0]
    y = utm.from_latlon(lat, long)[1]
    z = alt
    zone_number = utm.from_latlon(lat, long)[2]
    zone_letter = utm.from_latlon(lat, long)[3]
    return x, y, z, zone_number, zone_letter


def RSSI_Loc_BatchedGD_5par(data, numEpoch, numIterations, threshold, learning_rate):
    # init
    gradient = np.zeros(5)
    theta_all = np.zeros(shape=(numEpoch, 5))
    cost_all = np.zeros(numEpoch)

    m = len(data)
    # to UTM coordinate
    [x, y, z, zone_number, zone_letter] = GPS2UTM(data)

    # move origin
    xx = x - x[0]
    yy = y - y[0]
    zz = z
    pos = [xx, yy, zz]  # pos: independent variables

    signal_power = []
    for i in range(0, len(data)):
        signal_power.append(data[i][3])
        p = np.array(signal_power)  # y: dependent variables

    for j in range(0, numEpoch):
        theta = np.array([-20.0, np.random.normal(29.0,1),np.random.normal(-95.0,1),np.random.normal(2,1),0.5])
        # theta = np.array([np.random.normal(-20.0,1), np.random.normal(0,50),np.random.normal(0,50),np.random.normal(2,1),np.random.normal(0,1)])
        for i in range(0, numIterations):
            hypothesis = theta[0] * np.log10(np.sqrt(
                (theta[1] - pos[0]) ** 2 + (theta[2] - pos[1]) ** 2 + (theta[3] - pos[2]) ** 2)) + \
                         theta[4]
            loss = hypothesis - p
            cost = np.sum(loss ** 2) / (2 * m)
            # print("Iteration %d | Cost: %f" % (i, cost))
            if i > 0 and np.abs(cost - cost_pre) < threshold:
                break
            if math.isnan(cost):
                break
            # gradient
            gradient[0] = np.sum(loss * np.log10(np.sqrt(
                (theta[1] - x[0]) ** 2 + (theta[2] - x[1]) ** 2 + (theta[3] - x[2]) ** 2))) / m
            gradient[1] = np.sum(loss * theta[0] * (theta[1] - xx) / (
                    ((theta[1] - xx) ** 2 + (theta[2] - yy) ** 2 + (theta[3] - zz) ** 2) * np.log(
                    10))) / m
            gradient[2] = np.sum(loss * theta[0] * (theta[2] - xx) / (
                    ((theta[1] - xx) ** 2 + (theta[2] - yy) ** 2 + (theta[3] - zz) ** 2) * np.log(
                    10))) / m
            gradient[3] = np.sum(loss * theta[0] * (theta[3] - xx) / (
                    ((theta[1] - xx) ** 2 + (theta[2] - yy) ** 2 + (theta[3] - zz) ** 2) * np.log(
                    10))) / m
            gradient[4] = np.sum(loss) / m
            # update
            theta = theta - learning_rate * gradient
            cost_pre = cost
        theta_all[j] = theta
        cost_all[j] = cost

    # Tx location in xyz coordinates.
    result = np.ndarray.tolist(theta_all[np.where(cost_all == cost_all.min())][0])

    # convert Tx coordinates back to {lat, long, alt}
    result_latlonalt = np.zeros(3)
    [result_latlonalt[0], result_latlonalt[1]] = utm.to_latlon(x[0] + result[1], y[0] + result[2],
                                                               zone_number, zone_letter)
    result_latlonalt[2] = result[3]
    result_rssi_model = [result[0], result[4]]
    return result_latlonalt, result_rssi_model
