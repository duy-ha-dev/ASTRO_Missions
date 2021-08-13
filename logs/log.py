import pandas as pd

import numpy as np

###Goal: Set program so that script takes in file name as a paramter!!###


#Delimiters for logger files: ": " and ", "
#So our sep parameter would be sep='[:,]'

#Read the file
def main():
    print(' *** Using pandas.read_csv() with Custom delimiter *** ')

    # Read a csv file to a dataframe with custom delimiter
    df =  pd.read_csv('baseball_hover_log10m.csv', sep='[:,]'  , engine='python')

    #Label columns
    df.columns = ["", "pitch", "", "lat", "", "timestamp", "", "alt", "", "yaw", "", "lon", "", "heading", "", "speed", "", "roll"]

    #Drop non-numerical valued columns
    df.drop('', inplace=True, axis=1)

    print('Contents of Dataframe : ')
    print(df)


if __name__ == '__main__':
    main()