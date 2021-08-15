#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr  9 17:46:58 2020

@author: Tarence
"""

import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.neighbors import KNeighborsClassifier
from sklearn.model_selection import cross_val_score
import numpy as np
from sklearn.model_selection import GridSearchCV
from numpy import diff
from numpy import savetxt
import matplotlib.pyplot as plt
from sklearn.svm import SVC
from sklearn.metrics import accuracy_score

import matplotlib.pyplot as plt
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis as LDA
from sklearn.metrics import confusion_matrix, classification_report
from sklearn.model_selection import cross_val_score

import seaborn as sns
import scipy.stats as stats

# Read files
phase = pd.read_csv('pole_exp_1_clean/goldline/h5/phase_first_d.csv', index_col=[0])
amplitude = pd.read_csv('pole_exp_1_clean/goldline/h5/amp_first_d.csv', index_col=[0])
complex_h = pd.read_csv('pole_exp_1_clean/goldline/h5/complex_first_d.csv', index_col=[0])


# Make values data frame to float
phase = phase.values
amplitude = amplitude.values
complex_h = complex_h.values

# =============================================================================
# To do every other
# =============================================================================
phase_odd = phase[::2]
phase_even = phase[1::2]

amplitude_odd = amplitude[::2]
amplitude_even = amplitude[1::2]


# =============================================================================
# Time & select atennas
# =============================================================================

#time 8 sec for 2000
time = np.arange(0,8,(8/len(phase[0])))


# Select atennnas
phase_1 = phase_odd[:8]
phase_2 = phase_odd[8:16]

amplitude_1 = amplitude_odd[:8]
amplitude_2 = amplitude_odd[8:16]


# ever other amplitude 
amplitude_odd = amplitude_odd[:]
amplitude_even = amplitude_even[:]


# ever other amplitude 
phase_odd = phase_odd[:]
phase_even = phase_even[:]



# phase_1 = phase_1.flatten()  
# phase_2 = phase_2.flatten()  

# =============================================================================
# Plot single amp and phase
# =============================================================================

# def plot(time, phase, title = 'Needed'):
#     plt.plot(time, phase.T)
#     plt.title('{} Total'.format(title '))
#     plt.xlabel('Samples')
#     plt.ylabel('Phase in degress from +-180')
#     plt.show()
####
# All
plt.plot(time, phase.T)
plt.title('Phase Total')
plt.xlabel('Seconds')
plt.ylabel('Phase in degress from +-180')
plt.show()


plt.plot(time, amplitude.T)
plt.title('Amplitude Total')
plt.xlabel('Seconds')
plt.ylabel('Amplitude')
plt.show()

####
# Only the ones needed
plt.plot(time, phase_1.T)
plt.title('Phase First Row')
plt.xlabel('Seconds')
plt.ylabel('Phase in degress from +-180')
plt.show()


plt.plot(time, amplitude_1.T)
plt.title('Amplitude First Row')
plt.xlabel('Seconds')
plt.ylabel('Amplitude')
plt.show()



# Only the second needed
plt.plot(time, phase_2.T)
plt.title('Phase Second Row')
plt.xlabel('Seconds')
plt.ylabel('Phase in degress from +-180')
plt.show()


plt.plot(time, amplitude_2.T)
plt.title('Amplitude Second Row')
plt.xlabel('Seconds')
plt.ylabel('Amplitude')
plt.show()

# 
plt.plot(time, amplitude_odd.T)
plt.title('Amplitude Chain A')
plt.xlabel('Seconds')
plt.ylabel('Amplitude')
plt.show()


plt.plot(time, amplitude_even.T)
plt.title('Amplitude Chain B')
plt.xlabel('Seconds')
plt.ylabel('Amplitude')
plt.show()


# 
plt.plot(time, phase_odd.T)
plt.title('Phase Chain A')
plt.xlabel('Seconds')
plt.ylabel('Phase in degress from +-180')
plt.show()


plt.plot(time, phase_even.T)
plt.title('Phase Chain B')
plt.xlabel('Seconds')
plt.ylabel('Phase in degress from +-180')
plt.show()



# =============================================================================
# Correlation Functions: “Correlation” is a statistical term describing the degree to which two variables move in coordination with one-another.
# =============================================================================

# amplitude_corr = np.correlate(amplitude_1, amplitude_2, "same")
# print('Amplitude correlate')
# print(amplitude_corr)

# phase_corr = np.correlate(phase_1 , phase_2)

# #plt.scatter(amplitude_corr) 
# plt.plot(amplitude_corr)
# plt.title('Correlation of amplitude')
# plt.xlabel('time (s)')
# plt.ylabel('amplidude')
# plt.show()
def plot(plot_1, plot_2, time , x) :
    #plt.scatter(range(0, len(corr)), corr) 
    plt.plot(time, plot_1)
    plt.plot(time, plot_2)
    plt.title(x)
    plt.xlabel('time (s)')
    plt.ylabel(x)
    plt.show()

def correlation(corr_1, corr_2, time, x) :
    corr = []
    for i in range(8):
        corr = np.correlate(corr_1, corr_2, "same")
    
    tau, p_value = stats.kendalltau(corr_1, corr_2)
    #corr = np.corrcoef(corr_1, corr_2, rowvar=False)
    print(tau)
    plt.plot(time, corr)
    plt.title('Correlation')
    plt.xlabel('time (s)')
    plt.ylabel(x)
    plt.show()
    return corr
    
    
def correlation_2(corr_1, time, x) :
    #corr = np.corrcoef(corr_1, rowvar=False)
    corr = np.corrcoef(corr_1)
    print(np.shape(corr))
    plt.plot(corr)
    plt.title(x)
    plt.xlabel('time (s)')
    plt.ylabel(x)
    plt.show()
    
    fig, ax = plt.subplots()
    im = ax.imshow(corr, origin='lower')
    im.set_clim(-1, 1)
    ax.figure.colorbar(im, ax=ax, format='% .2f')
    ax.grid(False)
    ax.set_title('Heat Map of Correlation Amp Even')
    ax.set_xlabel('Antennas')
    ax.set_ylabel('Antennas')
    
    # ax.xaxis.set( ticklabels=('x', '30', '31','32', '33', '34', '35', '36'))
    #ax.yaxis.set( ticklabels=('48', '45','4', '3', '2', '1', '0'))
    # ax.xaxis.set(ticks=(0, 1, 2), ticklabels=('x', 'y', 'z'))
    # ax.yaxis.set(ticks=(0, 1, 2), ticklabels=('x', 'y', 'z'))
    #ax.set_ylim(2.5, -0.5)
    # for i in range(len(corr)):
    #     for j in range(len(corr)):
    #         ax.text(j, i, corr[i, j], ha='center', va='center',
    #             color='r')
    # cbar = ax.figure.colorbar(im, ax=ax, format='% .2f')
    plt.show()
    return corr
    


#One vs one over space average 

# Taking the mean
x_1 = amplitude_odd.mean(1)

x_1 = x_1.flatten()

corr_2 = correlation_2(amplitude_odd, time, x = "Amplitude Odd")
#corr_2 = correlation_2(x_1, time, x = "Amplitude Odd")

corr_2 = correlation_2(amplitude_even, time, x = "Amplitude Even")

# # FLatten Make (1,2000) to (2000,)
# amplitude_1 = amplitude_1.flatten() 
# amplitude_2 = amplitude_2.flatten() 

# #corr = correlation(amplitude_1, amplitude_2, time, x = "Amplitude")


# # Time 8 seconds x 8 for length of atennas
# time = np.arange(0,8 * 8,(8/len(phase[0])))

# # #One vs one over time
# plot(amplitude_1, amplitude_2, time, x = "amplitude")
# correlation(amplitude_1, amplitude_2, time, x = "amplitude")


# # Look here for phase information
# # =============================================================================
# # https://towardsdatascience.com/four-ways-to-quantify-synchrony-between-time-series-data-b99136c4a9c9
# # =============================================================================
# plot(phase_1, phase_2, time, x = "phase")
# correlation(phase_1, phase_2, time, x = "phase")


# # =============================================================================
# # Autocorrelation

# # Best: https://www.kaggle.com/selfishgene/filtering-and-auto-correlation-tutorial

# # =============================================================================
# # importing and creating alias for seaborn 

# def acf(x, length=1999):
#     #x = np.transpose(x)
#     return np.array([1]+[np.corrcoef(x[:-i], x[i:])[0,1]  \
#         for i in range(1, length)])
# ###        
# amplitude = amplitude.flatten() 
# phase = phase.flatten()  

# ##
     
# amplitude = np.transpose(amplitude)        
# x = acf(amplitude)
# plt.plot(x)
# plt.xlabel('Lag (index)')
# plt.ylabel('Autocorrelation')
# plt.title('1Autocorrelation')
# plt.show()
# ###
# ###
# import numpy as np

# from statsmodels.graphics import utils
# from statsmodels.tsa.stattools import acf, pacf


# def plot_acf(x, ax=None, lags=None, alpha=.05, use_vlines=True, unbiased=False,
#              fft=False, title='Autocorrelation', zero=True,
#              vlines_kwargs=None, **kwargs):
#     """Plot the autocorrelation function

#     Plots lags on the horizontal and the correlations on vertical axis.

#     Parameters
#     ----------
#     x : array_like
#         Array of time-series values
#     ax : Matplotlib AxesSubplot instance, optional
#         If given, this subplot is used to plot in instead of a new figure being
#         created.
#     lags : int or array_like, optional
#         int or Array of lag values, used on horizontal axis. Uses
#         np.arange(lags) when lags is an int.  If not provided,
#         ``lags=np.arange(len(corr))`` is used.
#     alpha : scalar, optional
#         If a number is given, the confidence intervals for the given level are
#         returned. For instance if alpha=.05, 95 % confidence intervals are
#         returned where the standard deviation is computed according to
#         Bartlett's formula. If None, no confidence intervals are plotted.
#     use_vlines : bool, optional
#         If True, vertical lines and markers are plotted.
#         If False, only markers are plotted.  The default marker is 'o'; it can
#         be overridden with a ``marker`` kwarg.
#     unbiased : bool
#         If True, then denominators for autocovariance are n-k, otherwise n
#     fft : bool, optional
#         If True, computes the ACF via FFT.
#     title : str, optional
#         Title to place on plot.  Default is 'Autocorrelation'
#     zero : bool, optional
#         Flag indicating whether to include the 0-lag autocorrelation.
#         Default is True.
#     vlines_kwargs : dict, optional
#         Optional dictionary of keyword arguments that are passed to vlines.
#     **kwargs : kwargs, optional
#         Optional keyword arguments that are directly passed on to the
#         Matplotlib ``plot`` and ``axhline`` functions.

#     Returns
#     -------
#     fig : Matplotlib figure instance
#         If `ax` is None, the created figure.  Otherwise the figure to which
#         `ax` is connected.

#     See Also
#     --------
#     matplotlib.pyplot.xcorr
#     matplotlib.pyplot.acorr

#     Notes
#     -----
#     Adapted from matplotlib's `xcorr`.

#     Data are plotted as ``plot(lags, corr, **kwargs)``

#     kwargs is used to pass matplotlib optional arguments to both the line
#     tracing the autocorrelations and for the horizontal line at 0. These
#     options must be valid for a Line2D object.

#     vlines_kwargs is used to pass additional optional arguments to the
#     vertical lines connecting each autocorrelation to the axis.  These options
#     must be valid for a LineCollection object.

#     Examples
#     --------
#     >>> import pandas as pd
#     >>> import matplotlib.pyplot as plt
#     >>> import statsmodels.api as sm

#     >>> dta = sm.datasets.sunspots.load_pandas().data
#     >>> dta.index = pd.Index(sm.tsa.datetools.dates_from_range('1700', '2008'))
#     >>> del dta["YEAR"]
#     >>> sm.graphics.tsa.plot_acf(dta.values.squeeze(), lags=40)
#     >>> plt.show()

#     .. plot:: plots/graphics_tsa_plot_acf.py
#     """
#     fig, ax = utils.create_mpl_ax(ax)

#     lags, nlags, irregular = _prepare_data_corr_plot(x, lags, zero)
#     vlines_kwargs = {} if vlines_kwargs is None else vlines_kwargs

#     confint = None
#     # acf has different return type based on alpha
#     if alpha is None:
#         acf_x = acf(x, nlags=nlags, alpha=alpha, fft=fft,
#                     unbiased=unbiased)
#     else:
#         acf_x, confint = acf(x, nlags=nlags, alpha=alpha, fft=fft,
#                              unbiased=unbiased)

#     _plot_corr(ax, title, acf_x, confint, lags, irregular, use_vlines,
#                vlines_kwargs, **kwargs)

#     return fig


# def _prepare_data_corr_plot(x, lags, zero):
#     zero = bool(zero)
#     irregular = False if zero else True
#     if lags is None:
#         # GH 4663 - use a sensible default value
#         nobs = x.shape[0]
#         lim = min(int(np.ceil(10 * np.log10(nobs))), nobs - 1)
#         lags = np.arange(not zero, lim + 1)
#     elif np.isscalar(lags):
#         lags = np.arange(not zero, int(lags) + 1)  # +1 for zero lag
#     else:
#         irregular = True
#         lags = np.asanyarray(lags).astype(np.int)
#     nlags = lags.max(0)

#     return lags, nlags, irregular


# def _plot_corr(ax, title, acf_x, confint, lags, irregular, use_vlines,
#                vlines_kwargs, **kwargs):
#     if irregular:
#         acf_x = acf_x[lags]
#         if confint is not None:
#             confint = confint[lags]

#     if use_vlines:
#         ax.vlines(lags, [0], acf_x, **vlines_kwargs)
#         ax.axhline(**kwargs)

#     kwargs.setdefault('marker', 'o')
#     kwargs.setdefault('markersize', 5)
#     if 'ls' not in kwargs:
#         # gh-2369
#         kwargs.setdefault('linestyle', 'None')
#     ax.margins(.05)
#     ax.plot(lags, acf_x, **kwargs)
#     ax.set_title(title)

#     if confint is not None:
#         if lags[0] == 0:
#             lags = lags[1:]
#             confint = confint[1:]
#             acf_x = acf_x[1:]
#         lags = lags.astype(np.float)
#         lags[0] -= 0.5
#         lags[-1] += 0.5
#         ax.fill_between(lags, confint[:, 0] - acf_x,
#                         confint[:, 1] - acf_x, alpha=.25)






# ###
# ###
# # https://medium.com/@krzysztofdrelczuk/acf-autocorrelation-function-simple-explanation-with-python-example-492484c32711
# #from statsmodels.graphics.tsaplots import plot_acf
# #plot_acf(phase, lags=100)
# plot_acf(amplitude, lags=1999, )
# plt.xlabel('Lag (index)')
# plt.ylabel('Autocorrelation')
# plt.title('Autocorrelation')
# plt.show()
# ###
# ###
