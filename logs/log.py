import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from optparse import OptionParser
import sys

###Goal: Set program so that script takes in file name as a paramter!!###

#Variables
TICK_INTERVAL = .5

#Initialization
filename = sys.argv[1]
parser = OptionParser()

#Delimiters for logger files: ": " and ", "
#So our sep parameter would be sep='[:,]'

#Read the file
def main():
    #Close any current plots opened
    plt.close("all")

    print(' *** Using pandas.read_csv() with Custom delimiter *** ')

    # Read a csv file to a dataframe with custom delimiter
    df =  pd.read_csv(filename, sep='[:,]'  , engine='python', skipinitialspace=True)

    #Label columns
    variables_list = ["", "PITCH", "", "LAT", "", "TIMESTAMP", "", "ALT", "", "YAW", "", "LON", "", "HEADING", "", "SPEED", "", "ROLL"]
    df.columns = variables_list
    #Remove "" in our list of variables
    variables_list = [i for i in variables_list if i != ""]

    #Change row index as to match the time
    new_index = {}
    tick_counter = 0
    for idx in range(len(df)):
        tick_counter += TICK_INTERVAL
        new_index[idx] = tick_counter
    df = df.rename(index=new_index)

    #Drop non-numerical valued columns
    df.drop('', inplace=True, axis=1)
    #Changing all column variables to float type
    for variable in variables_list:
        if df.dtypes[variable] == np.object:
            #Remove special characters from column values; these specific characters are seen in the logs
            df[variable] = df[variable].str.replace(' ', '')
            df[variable] = df[variable].str.replace('"', '')
            df[variable] = df[variable].str.replace('}', '')

            df[variable] = pd.to_numeric(df[variable], downcast='float')
            print(df[variable][0.5])
        df[variable] = pd.to_numeric(df[variable], downcast='float', errors='coerce')

    #Display dataframe
    print('Contents of Dataframe : ')
    print(df)
    print(type(df))

    #PRINTING OUT LOG PLOTS ONE BY ONE
    #Turn columns of dataframe into one-dimensional ndarray: Seriesframe
    #We store each seriesframe in a dictionary sf, ex.) print(sf['lat']) will give us the 1-d ndarray for lat
    sf = {}
    for variable in variables_list:
        sf[variable] = df[variable].squeeze()


    #Plotting
    variables_list_copy = variables_list.copy()
    variables_list_copy.remove('timestamp') #Remove timestamp


    for variable in variables_list_copy:
        sf[variable].plot(kind='line', use_index=True, title='{} VS TIME PLOT'.format(variable), grid=True,
                            xlabel='TIME', ylabel=variable)
        plt.show()


if __name__ == '__main__':
    main()