import ast
import re
import numpy as np
import datetime as dt
import time
import math
import utm

import numpy as np
import importlib
import math
import random
import utm
import time
from gradient_descent import RSSI_Loc_BatchedGD_5par
from gradient_descent import GPS2UTM
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error
import scipy.optimize as opt
from sklearn.cluster import KMeans
from astro_util import IntersectPoints
from astro_util import closestpair


EST_CHEATER_HEIGHT = 1.0 # Meters

# class used to using algorithm to calculate error, change parameters when initializing according to your own
# requirements

class predict:
    def __init__(self, data_length, dronenum, maxRun=1, numIterations = 10, numEpoch = 500, threshold = 0.1,
                 learning_rate = 0.001, numberBatch = 1):
        self.num = dronenum
        self.maxRun = maxRun
        self.numIterations = numIterations
        self.numEpoch = numEpoch
        self.threshold = threshold
        self.learning_rate = learning_rate
        self.numberBatch = numberBatch
        self.BatchSize_min = data_length
        self.BatchSize_max = data_length
        self.BatchSize_step = data_length

    def swarm(self, drone_data, alphas, epsilons):
        # print(drone_data)
        # print(len(drone_data))
        data_drone1 = np.array(drone_data[0])
        data_drone2 = np.array(drone_data[1])
        if len(drone_data) > 2:
            data_drone3 = np.array(drone_data[2])
        else:
            data_drone3 = data_drone2

        if self.num == 2:
            ###RUN 2 DRONES
            derror = []
            error_2drones = []
            error_2drones_tot = []
            for index in range(0, self.maxRun):

                # alpha1 = alpha
                # epsilon1 = epsilon
                # alpha2 = alpha
                # epsilon2 = epsilon

                centers = []
                errorsBlackBox_2d = []
                [x1_org, y1_org, z1_org, zone_number, zone_letter] = GPS2UTM(data_drone1[:, 0:3])
                [x2_org, y2_org, z2_org, zone_number, zone_letter] = GPS2UTM(data_drone2[:, 0:3])

          #       for i_batch in range(0, self.numberBatch):

          #           data_drone1_test = data_drone1[np.int(i_batch * self.BatchSize_step):np.int(
          #               i_batch * self.BatchSize_step + self.BatchSize_min)]
          #           data_drone2_test = data_drone2[np.int(i_batch * self.BatchSize_step):np.int(
          #               i_batch * self.BatchSize_step + self.BatchSize_min)]

          #           # #     # convert GPS to UTM coordinate
          #           [x1, y1, z1, zone_number, zone_letter] = GPS2UTM(data_drone1_test)
          #           [x2, y2, z2, zone_number, zone_letter] = GPS2UTM(data_drone2_test)

          #           # #     # move origin
          #           xx1 = x1 - x1_org[0]
          #           yy1 = y1 - y1_org[0]

          #           xx2 = x2 - x2_org[0]
          #           yy2 = y2 - y2_org[0]

          #           result = []

          #           for i in range(0, len(data_drone1_test)):
          #               result_sub = []

          #               r1 = 10 ** ((data_drone1_test[i][3] - epsilons[0]) / alphas[0])
          #               r2 = 10 ** ((data_drone2_test[i][3] - epsilons[1]) / alphas[1])

          #               sol1 = IntersectPoints(complex(xx1[i], yy1[i]), complex(xx2[i], yy2[i]), r1, r2)

          #               if sol1 is True:
          #                   pass
          #               elif sol1 is False:
          #                   pass
          #               else:
          #                   result_sub.append((sol1[0].real, sol1[0].imag))
          #                   result_sub.append((sol1[1].real, sol1[1].imag))

          #               if result_sub:
          #                   temp = closestpair(result_sub)
          #                   result.append([(temp[0][0] + temp[1][0]) / 2, (temp[0][1] + temp[1][1]) / 2])

          #           if result:
          #               kmeans = KMeans(n_clusters=2)
          #               kmeans.fit(result)
          #               centers.append(kmeans.cluster_centers_)
          #           else:
          #               try:
          #                   centers.append(centers[-1])
          #               except IndexError:
          #                   pass

          #           result_latlonalt = np.zeros(shape=(len(centers), 3))

          #           x_org = (x1_org[0] + x2_org[0]) / 2
          #           y_org = (y1_org[0] + y2_org[0]) / 2

          #       for i in range(0, len(centers)):
          #       	[result_latlonalt[i][0], result_latlonalt[i][1]] = utm.to_latlon(centers[i][0][0] + x_org,
          #                                                                                centers[i][0][1] + y_org,
          #                                                                                zone_number, zone_letter)
    	    	# return result_latlonalt[0]

        else:
            #########3DRONEs
            derror = []
            error_3drones = []
            error_3drones_tot = []
            # Find height above drones so that the radius calculation can be done in 2D
            drone1_alt = data_drone1[0][2] - EST_CHEATER_HEIGHT
            drone2_alt = data_drone2[0][2] - EST_CHEATER_HEIGHT
            drone3_alt = data_drone3[0][2] - EST_CHEATER_HEIGHT

            for index in range(0, self.maxRun):

                centers = []
                errorsBlackBox_2d = []

                [x1_org, y1_org, z1_org, zone_number, zone_letter] = GPS2UTM(data_drone1[:, 0:3])
                [x2_org, y2_org, z2_org, zone_number, zone_letter] = GPS2UTM(data_drone2[:, 0:3])
                [x3_org, y3_org, z3_org, zone_number, zone_letter] = GPS2UTM(data_drone3[:, 0:3])

                x_org = (x1_org[0] + x2_org[0] + x3_org[0]) / 3
                y_org = (y1_org[0] + y2_org[0] + y3_org[0]) / 3

                for i_batch in range(0, self.numberBatch):

                    data_drone1_test = data_drone1[np.int(i_batch * self.BatchSize_step):np.int(
                        i_batch * self.BatchSize_step + self.BatchSize_min)]
                    data_drone2_test = data_drone2[np.int(i_batch * self.BatchSize_step):np.int(
                        i_batch * self.BatchSize_step + self.BatchSize_min)]
                    data_drone3_test = data_drone3[np.int(i_batch * self.BatchSize_step):np.int(
                        i_batch * self.BatchSize_step + self.BatchSize_min)]

                    # convert GPS to UTM coordinate
                    [x1, y1, z1, zone_number, zone_letter] = GPS2UTM(data_drone1_test)
                    [x2, y2, z2, zone_number, zone_letter] = GPS2UTM(data_drone2_test)
                    [x3, y3, z3, zone_number, zone_letter] = GPS2UTM(data_drone3_test)

                    # move origin
                    xx1 = x1 - x_org
                    yy1 = y1 - y_org

                    xx2 = x2 - x_org
                    yy2 = y2 - y_org

                    xx3 = x3 - x_org
                    yy3 = y3 - y_org

                    result = []

                    for i in range(0, len(data_drone1_test)):
                        result_sub = []

                        # Estimated 3D distance of Tx to drone
                        d1 = 10 ** ((data_drone1_test[i][3] - epsilons[0]) / alphas[0])
                        d2 = 10 ** ((data_drone2_test[i][3] - epsilons[1]) / alphas[1])
                        d3 = 10 ** ((data_drone3_test[i][3] - epsilons[2]) / alphas[2])
                        # print d1, d2, d3

                        # 3D distance should always be greater than altitude. If not then the estimate is probably wrong
                        if d1 > drone1_alt and d2 > drone2_alt and d3 > drone3_alt:
                            r1 = math.sqrt(d1**2 - drone1_alt**2)
                            r2 = math.sqrt(d2**2 - drone2_alt**2)
                            r3 = math.sqrt(d3**2 - drone3_alt**2)
                            # print(r1, r2, r3)
                        else:
                            # If the estimated distances are too small to remove altiturde. just use the estimated distances
                            # TODO: try removing the altitude/2 maybe
                            r1 = d1
                            r2 = d2
                            r3 = d3

                        # print(complex(xx1[i], yy1[i]))
                        # print(complex(xx2[i], yy2[i]))
                        # print(complex(xx3[i], yy3[i]))

                        sol1 = IntersectPoints(complex(xx1[i], yy1[i]), complex(xx2[i], yy2[i]), r1, r2)
                        sol2 = IntersectPoints(complex(xx2[i], yy2[i]), complex(xx3[i], yy3[i]), r2, r3)
                        sol3 = IntersectPoints(complex(xx1[i], yy1[i]), complex(xx3[i], yy3[i]), r1, r3)

                        # print(sol1, sol2, sol3)

                        if sol1 is True:
                            pass
                        elif sol1 is False:
                            pass
                        else:
                            result_sub.append((sol1[0].real, sol1[0].imag))
                            result_sub.append((sol1[1].real, sol1[1].imag))

                        if sol2 is True:
                            pass
                        elif sol2 is False:
                            pass
                        else:
                            result_sub.append((sol2[0].real, sol2[0].imag))
                            result_sub.append((sol2[1].real, sol2[1].imag))

                        if sol3 is True:
                            pass
                        elif sol3 is False:
                            pass
                        else:
                            result_sub.append((sol3[0].real, sol3[0].imag))
                            result_sub.append((sol3[1].real, sol3[1].imag))

                        if result_sub:
                            temp = closestpair(result_sub)
                            result.append([(temp[0][0] + temp[1][0]) / 2, (temp[0][1] + temp[1][1]) / 2])
                    # if result:
                    #     # print(result)
                    #     # Try to run KMeans with 3 clusters, if that doesn't work reduce cluster count until it does
                    #     did_work = True
                    #     for cluster_count in range(3, 0, -1):
                    #         try:
                    #             kmeans = KMeans(n_clusters=cluster_count)
                    #             kmeans.fit(result)
                    #         except ValueError:
                    #             did_work = False

                    #         if did_work:
                    #             centers.append(kmeans.cluster_centers_)
                    #             break

                    if result:
                        kmeans = KMeans(n_clusters=1)
                        kmeans.fit(result)
                        centers.append(kmeans.cluster_centers_)

                    else:
                        print "NO RESULTS"
                        try:
                            centers.append(centers[-1])
                        except IndexError:
                            pass

                    result_latlonalt = np.zeros(shape=(len(centers), 3))

            	for i in range(0, len(centers)):

                	[result_latlonalt[i][0], result_latlonalt[i][1]] = utm.to_latlon(centers[i][0][0] + x_org,
                                                                                         centers[i][0][1] + y_org,
                                                                                         zone_number, zone_letter)
    	    	return result_latlonalt[0]


def haversine(lat1, lon1, lat2, lon2):
   """
   Calculate the great circle distance between two points
   on the earth (specified in decimal degrees)
   """

   # convert decimal degrees to radians

   lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])

   # haversine formula
   dlon = lon2 - lon1
   dlat = lat2 - lat1
   a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
   c = 2 * math.asin(math.sqrt(a))

   km = EARTH_RADIUS_KM * c
   return km * 1000


def mini(a, b):
    if a <= b:
        return a
    else:
        return b


def stddev(lst):
    """returns the standard deviation of lst"""
    variance = 0
    mn = sum(lst)/len(lst)
    for e in lst:
        variance += (e-mn)**2
    variance /= len(lst)

    if (variance == 0):
        return 1.0
    else:
        return sqrt(variance)


def readdata(file):
    # this function used to read data from the log file
    in_file = open(file, 'rt')
    contents = in_file.read()  # read the entire file into a string variable
    in_file.close()
    strs = contents

    #   Getting the strings dBm, lat, and lon values from the huge text file
    #   time_values_raw = re.findall(r"'time_stamp': '[+-]?\d+(?:\.\d+)?'", strs)
    dBm_values_raw = re.findall(r"\"dBm\": [+-]?\d+(?:\.\d+)?", strs)
    lat_values_raw = re.findall(r"\"lat\": [+-]?\d+(?:\.\d+)?", strs)
    lon_values_raw = re.findall(r"\"lon\": [+-]?\d+(?:\.\d+)?", strs)
    alt_values_raw = re.findall(r"\"alt\": [+-]?\d+(?:\.\d+)?", strs)

    #   Making the lists into giant strings
    dBm_values_raw_string = ''.join(dBm_values_raw)
    lat_values_raw_string = ''.join(lat_values_raw)
    lon_values_raw_string = ''.join(lon_values_raw)
    alt_values_raw_string = ''.join(alt_values_raw)

    #   Finding the numbers in the giant string to filter out those phrases mentioned above
    dBm = re.findall(r"[+-]?\d+(?:\.\d+)?", dBm_values_raw_string)
    lat = re.findall(r"[+-]?\d+(?:\.\d+)?", lat_values_raw_string)
    lon = re.findall(r"[+-]?\d+(?:\.\d+)?", lon_values_raw_string)
    alt = re.findall(r"[+-]?\d+(?:\.\d+)?", alt_values_raw_string)

    #   Floating all the data
    dBm_data = [float(i) for i in dBm]
    lat_data = [float(k) for k in lat]
    lon_data = [float(j) for j in lon]
    alt_data = [float(z) for z in alt]

    drone_coords_data = zip(lat_data, lon_data, alt_data, dBm_data)

    return drone_coords_data


def altfilter(alt, drone, tolerance):
    # used to filter altitudes
    filterdata = []
    for i in range(0, len(drone)):
        if alt-tolerance <= drone[i][2] <= alt+tolerance:
            filterdata.append(drone[i])
    return filterdata


def translation(size, drone1, drone2, drone3):
    data_drone1 = np.zeros(shape=(size, 4))
    data_drone2 = np.zeros(shape=(size, 4))
    data_drone3 = np.zeros(shape=(size, 4))
    for i in range(0, size):
        data_drone1[i][0] = drone1[i][0]
        data_drone1[i][1] = drone1[i][1]
        data_drone1[i][2] = drone1[i][2]
        data_drone1[i][3] = drone1[i][3]

        data_drone2[i][0] = drone2[i][0]
        data_drone2[i][1] = drone2[i][1]
        data_drone2[i][2] = drone2[i][2]
        data_drone2[i][3] = drone2[i][3]

        data_drone3[i][0] = drone3[i][0]
        data_drone3[i][1] = drone3[i][1]
        data_drone3[i][2] = drone3[i][2]
        data_drone3[i][3] = drone3[i][3]
    return data_drone1, data_drone2, data_drone3
