#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun May 16 20:28:18 2021

@author: red
"""

#!/usr/bin/python3
"""
 plot_hdf5.py

 Plotting from HDF5 file
 Script to analyze recorded hdf5 file from channel sounding (see Sounder/).
 Usage format is:
    ./plot_hdf5.py <hdf5_file_name>

 Example:
    ./plot_hdf5.py ../Sounder/logs/test-hdf5.py


---------------------------------------------------------------------
 Copyright © 2018-2020. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import sys
import numpy as np
import h5py
import matplotlib.pyplot as plt
import collections
import time
from find_lts import *
from optparse import OptionParser
from channel_analysis import *
import hdf5_lib
from hdf5_lib import *
import matplotlib
#matplotlib.use("Agg")
## Tarence added
import csv
import pandas as pd
#


# Tested with inputs: ./data_in/Argos-2019-3-11-11-45-17_1x8x2.hdf5 300  (for two users)
#                     ./data_in/Argos-2019-3-30-12-20-50_1x8x1.hdf5 300  (for one user) 
parser = OptionParser()
parser.add_option("--show-metadata", action="store_true", dest="show_metadata", help="Displays hdf5 metadata", default=False)
parser.add_option("--deep-inspect", action="store_true", dest="deep_inspect", help="Run script without analysis", default=False)
parser.add_option("--ref-frame", type="int", dest="ref_frame", help="Frame number to plot", default=1000)
parser.add_option("--ref-ul-subframe", type="int", dest="ref_ul_subframe", help="Frame number to plot", default=0)
parser.add_option("--ref-cell", type="int", dest="ref_cell", help="Cell number to plot", default=0)
parser.add_option("--legacy", action="store_true", dest="legacy", help="Parse and plot legacy hdf5 file", default=False)
parser.add_option("--ref-ant", type="int", dest="ref_ant", help="Reference antenna", default=0)
parser.add_option("--exclude-bs-ants", type="string", dest="exclude_bs_nodes", help="Bs antennas to be excluded in plotting", default="")
parser.add_option("--ref-ofdm-sym", type="int", dest="ref_ofdm_sym", help="Reference ofdm symbol within a pilot", default=0)
parser.add_option("--ref-user", type="int", dest="ref_user", help="Reference User", default=0)
parser.add_option("--ref-subcarrier", type="int", dest="ref_subcarrier", help="Reference subcarrier", default=0)
parser.add_option("--signal-offset", type="int", dest="signal_offset", help="signal offset from the start of the time-domain symbols", default=-1)
parser.add_option("--downlink-calib-offset", type="int", dest="downlink_calib_offset", help="signal offset from the start of the time-domain symbols in downlink reciprocal calibration", default=288)
parser.add_option("--uplink-calib-offset", type="int", dest="uplink_calib_offset", help="signal offset from the start of the time-domain symbols in uplink reciprocal calibration", default=168)
parser.add_option("--n-frames", type="int", dest="n_frames_to_inspect", help="Number of frames to inspect", default=2000)
parser.add_option("--sub-sample", type="int", dest="sub_sample", help="Sub sample rate", default=1)
parser.add_option("--thresh", type="float", dest="thresh", help="Ampiltude Threshold for valid frames", default=0.001)
parser.add_option("--frame-start", type="int", dest="fr_strt", help="Starting frame. Must have set n_frames_to_inspect first and make sure fr_strt is within boundaries ", default=0)
parser.add_option("--verify-trace", action="store_true", dest="verify", help="Run script without analysis", default=True)
parser.add_option("--analyze-trace", action="store_true", dest="analyze", help="Run script without analysis", default=False)
parser.add_option("--corr-thresh", type="float", dest="corr_thresh",
                  help="Correlation threshold to exclude bad nodes",
                  default=0.00)
(options, args) = parser.parse_args()

show_metadata = options.show_metadata
deep_inspect = options.deep_inspect
n_frames_to_inspect = options.n_frames_to_inspect
ref_frame = options.ref_frame
ref_cell = options.ref_cell
ref_ofdm_sym = options.ref_ofdm_sym
ref_ant = options.ref_ant
ref_user = options.ref_user
ref_subcarrier = options.ref_subcarrier
ref_ul_subframe = options.ref_ul_subframe
signal_offset = options.signal_offset
downlink_calib_offset = options.downlink_calib_offset
uplink_calib_offset = options.uplink_calib_offset
thresh = options.thresh
fr_strt = options.fr_strt
verify = options.verify
analyze = options.analyze
sub_sample = options.sub_sample
legacy = options.legacy
corr_thresh = options.corr_thresh
exclude_bs_nodes_str = options.exclude_bs_nodes
exclude_bs_nodes = []
if len(exclude_bs_nodes_str) > 0:
    exclude_ant_ids = exclude_bs_nodes_str.split(',')
    exclude_bs_nodes = [int(i) for i in exclude_ant_ids]
# =============================================================================
# 1st
# =============================================================================
#filename = 'pole_exp_1/goldline/h5/first_d.hdf5'
#filename = 'pole_exp_1/ten_yard/h5/first_d.hdf5'
#filename = 'pole_exp_1/twenty_yard/h5/first_d.hdf5'

#filename = 'pole_exp_1/goldline/h5/first.hdf5'
#filename = 'pole_exp_1/ten_yard/h5/first.hdf5'
filename = 'pole_exp_1/twenty_yard/h5/first.hdf5'

scrpt_strt = time.time()

if n_frames_to_inspect == 0:
    print("WARNING: No frames_to_inspect given. Will process the whole dataset.") 

if ref_frame > n_frames_to_inspect:
    print("WARNING: Attempted to inspect a frame at an index larger than the no. of requested frames: ref_frame:{} >  n_frames_to_inspect:{}. ".format(
            ref_frame, n_frames_to_inspect))
    print("Setting the frame to inspect to 0")
    ref_frame = 0
 
if (ref_frame > fr_strt + n_frames_to_inspect) or (ref_frame < fr_strt) :
    print("WARNING: Attempted to inspect a frame at an index larger than the no. of requested frames +  or at an index smaller than the required start of the frames: ref_frame:{} > n_frames_to_inspect:{} or ref_frame:{} <  fr_strt:{}. ".format(
            ref_frame, n_frames_to_inspect, ref_frame, fr_strt))
    print("Setting the frame to inspect/plot to {}".format(fr_strt))
    ref_frame = fr_strt

print(">> frame to plot = {}, ref. ant = {}, ref. user = {}, no. of frames to inspect = {}, starting frame = {} <<".format(ref_frame, ref_ant, ref_user, n_frames_to_inspect, fr_strt))

# Instantiate
if legacy:
    # TODO: Needs to be thoroughly tested!
    # filename = 'ArgosCSI-96x8-2016-11-03-03-03-45_5GHz_static.hdf5'
    hdf5 = h5py.File(str(filename), 'r')
    compute_legacy(hdf5)
else:
    if show_metadata:
        hdf5 = hdf5_lib(filename)
        print(hdf5.metadata)
        pilot_samples = hdf5.pilot_samples
        uplink_samples = hdf5.uplink_samples
        noise_avail = len(noise_samples) > 0

        # Check which data we have available
        pilots_avail = len(pilot_samples) > 0
        ul_data_avail = len(uplink_samples) > 0
        if pilots_avail:
            print("HDF5 pilot data size:")
            print(pilot_samples.shape)
        if ul_data_avail:
            print("HDF5 uplink data size:")
            print(uplink_samples.shape)

    else:
        hdf5 = hdf5_lib(filename, n_frames_to_inspect, fr_strt, sub_sample)
        data = hdf5.data
        pilot_samples = hdf5.pilot_samples
        uplink_samples = hdf5.uplink_samples
        noise_samples = hdf5.noise_samples

        # Check which data we have available
        pilots_avail = len(pilot_samples) > 0
        ul_data_avail = len(uplink_samples) > 0
        noise_avail = len(noise_samples) > 0

        if pilots_avail:
            print("Found Pilots!")
            if ul_data_avail:
                print("Found Uplink Data")
            if noise_avail:
                print("Found Noise Samples!")
        else:
            if not ul_data_avail:
                raise Exception(' **** No pilots or uplink data found **** ')

        if verify:
            
            hdf5
            frame_i=100
            cell_i=0
            ofdm_sym_i=0
            ant_i =0
            user_i=0
            ul_sf_i=0
            subcarrier_i=10
            offset=-1
            dn_calib_offset=0
            up_calib_offset=0
            thresh=0.001
            deep_inspect=False
            corr_thresh=0.00
            exclude_bs_nodes=[]
            
            plt.close("all")

            # Retrieve attributes
            n_frm_end = hdf5.n_frm_end
            n_frm_st = hdf5.n_frm_st
            metadata = hdf5.metadata
            
            
            
            
            symbol_length = int(metadata['SYMBOL_LEN'])
            num_pilots = int(metadata['PILOT_NUM'])
            num_cl = int(metadata['CL_NUM'])
            prefix_len = int(metadata['PREFIX_LEN'])
            postfix_len = int(metadata['POSTFIX_LEN'])
            z_padding = prefix_len + postfix_len
            if offset < 0: # if no offset is given use prefix from HDF5
                offset = int(prefix_len)
            fft_size = int(metadata['FFT_SIZE'])
            cp = int(metadata['CP_LEN'])
            rate = int(metadata['RATE'])
            pilot_type = metadata['PILOT_SEQ_TYPE'].astype(str)[0]
            nonzero_sc_size = metadata['DATA_SUBCARRIER_NUM']
            ofdm_pilot = np.array(metadata['OFDM_PILOT'])
            reciprocal_calib = np.array(metadata['RECIPROCAL_CALIB'])
            symbol_length_no_pad = symbol_length - z_padding
            num_pilots_per_sym = ((symbol_length_no_pad) // len(ofdm_pilot))
            n_ue = num_cl
            # print('Metaaa    aaaaajkadkhjadkhgdkhdw \n')
            # print(metadata['OFDM_PILOT_F'])
            
            # print(metadata['OFDM_PILOT_F'].shape)
        
            all_bs_nodes = set(range(hdf5.pilot_samples.shape[3]))
            plot_bs_nodes = list(all_bs_nodes - set(exclude_bs_nodes))
            pilot_samples = hdf5.pilot_samples[:, :, :, plot_bs_nodes, :]
            ul_data_avail = len(hdf5.uplink_samples) > 0
            if ul_data_avail:
                uplink_samples = hdf5.uplink_samples[:, :, :, plot_bs_nodes, :]
            noise_avail = len(hdf5.noise_samples) > 0
            if noise_avail:
                noise_samples = hdf5.noise_samples[:, :, :, plot_bs_nodes, :]
        
            frm_plt = min(frame_i, pilot_samples.shape[0] + n_frm_st)
        
            # Verify frame_i does not exceed max number of collected frames
            ref_frame = min(frame_i - n_frm_st, pilot_samples.shape[0])
        
            print("symbol_length = {}, offset = {}, cp = {}, prefix_len = {}, postfix_len = {}, z_padding = {}, pilot_rep = {}".format(symbol_length, offset, cp, prefix_len, postfix_len, z_padding, num_pilots_per_sym))
        
            # pilot_samples dimensions:
            # ( #frames, #cells, #pilot subframes or cl ant sending pilots, #bs nodes or # bs ant, #samps per frame * 2 for IQ )
            num_cl_tmp = num_pilots  # number of UEs to plot data for
            num_frames = pilot_samples.shape[0]
            num_cells = pilot_samples.shape[1]
            num_bs_ants = pilot_samples.shape[3]
        
            samps_mat = np.reshape(
                    pilot_samples, (num_frames, num_cells, num_cl_tmp, num_bs_ants, symbol_length, 2))
            samps = (samps_mat[:, :, :, :, :, 0] +
                    samps_mat[:, :, :, :, :, 1]*1j)*2**-15
        
            # Correlation (Debug plot useful for checking sync)
            good_ants = []
            insp_ants = [] # antennas to be inspected
            if ant_i > num_bs_ants - 1:
                insp_ants = range(samps.shape[3])
            else:
                insp_ants = [ant_i]
            for i in insp_ants:
                amps = np.mean(np.abs(samps[:, 0, user_i, i, :]), axis=1)
                pilot_frames = [i for i in range(len(amps)) if amps[i] > thresh]
                if len(pilot_frames) > 0:
                    good_ants = good_ants + [i]
                else:
                    print("no valid frames where found in antenna %d. Decision threshold (average pilot amplitude) was %f" % (i, thresh))
           
        
            # Compute CSI from IQ samples
            # Samps: #Frames, #Cell, #Users, #Antennas, #Samples
            # CSI:   #Frames, #Cell, #Users, #Pilot Rep, #Antennas, #Subcarrier
            # For correlation use a fft size of 64
            print("*verify_hdf5(): Calling samps2csi with fft_size = {}, offset = {}, bound = {}, cp = {} *".format(fft_size, offset, z_padding, cp))
            csi, samps = hdf5_lib.samps2csi(pilot_samples, num_cl_tmp, symbol_length, fft_size=fft_size, offset=offset, bound=z_padding,
                                        cp=cp, sub=1, pilot_type=pilot_type, nonzero_sc_size=nonzero_sc_size)
        # =============================================================================
        # Start    
        # =============================================================================
            user_plt = 0
        
            print('Printing CSI Data Shape    ################!!!!!!!!!!!!!!!!!')
            print(csi.shape)
            
            #array for amplitude vaules
            amplitude_value = []
            amplitude_value_csi = []
            
            #array for phase vaules
            phase_value = []
            phase_value_csi = []
            
            #array for raw vaules
            raw_samp = []
            raw_samp_csi = []
            raw_csi = []
            
            raw_samps = []
            
            
            for i in range(csi.shape[4]):
                    # Samps: #Frames, #Cell, #Users, #Pilot Rep, #Antennas, #Samples
                    #axes[4, idx].plot(np.mean(np.abs(samps[:, 0, user_plt, 0, i, :]), axis=1).flatten())
                    
                    ##Subcarrier 26
                    ##To check Calculator The polar form of complex numbers: https://www.hackmath.net/en/calculator/complex-number?input=10L60
                    
                    # Amplitude: https://numpy.org/doc/stable/reference/generated/numpy.absolute.html
                    #amplitude_value.append(np.mean(np.abs(samps[:, 0, user_plt, 0, i, :]), axis =1))
                    amplitude_value_csi.append(np.abs(csi[:, 0, user_plt, 0, i, 26]))
                    
                    # Phase: https://numpy.org/doc/stable/reference/generated/numpy.angle.html
                    #phase_value.append(np.mean(np.angle(samps[:, 0, user_plt, 0, i, :], deg=True), axis = 1))
                    phase_value_csi.append(np.angle(csi[:, 0, user_plt, 0, i, 26], deg=True))

                    # Raw: complex
                    raw_samp.append(samps[:, 0, user_plt, 0, i, 26].flatten())
                    raw_samp_csi.append(csi[:, 0, user_plt, 0, i, 26].flatten())
                    
                
                    # Average (mean) of all subcarriers
                    #raw_csi.append(np.mean(csi[:, 0, user_plt, 0, i, :], axis = 1))
                    #raw_samp.append(np.mean(samps[:, 0, user_plt, 0, i, :], axis = 1))
                    
# =============================================================================
# Second
# =============================================================================
            # # Convert to a dataframe and save to csv
            # # Only needed is CSI
            # amp_values_csi = pd.DataFrame(amplitude_value_csi, columns = [k for k in range(1,2001)])
            # # 2.1
            # amp_values_csi.to_csv('pole_exp_1_clean/goldline/h5/amp_first_d.csv')
            # print("^^^^^^^ Amplitude values saved")
            # #2.2   
            # phase_values_csi = pd.DataFrame(phase_value_csi, columns = [k for k in range(1,2001)])
            # phase_values_csi.to_csv('pole_exp_1_clean/goldline/h5/phase_first_d.csv')
            # print("^^^^^^^^^^^^^^ Phase values saved")
            # #2.3    
            # raw_samps_csi = pd.DataFrame(raw_samp_csi, columns = [k for k in range(1,2001)])
            # raw_samps_csi.to_csv('pole_exp_1_clean/goldline/h5/complex_first_d.csv',index=False)
            # print('^^^^^^^^^^^^^^^^^^^^^ Raw #1 (complex) samples saved ')
            #2.4    
            raw_samps_ = pd.DataFrame(raw_samp, columns = [k for k in range(1,2001)])
            # raw_samps_.to_csv('pole_exp_1_clean/goldline/h5/samps_complex_first_d.csv',index=False)
            # raw_samps_.to_csv('pole_exp_1_clean/ten_yard/h5/samps_complex_first_d.csv',index=False)
            # raw_samps_.to_csv('pole_exp_1_clean/twenty_yard/h5/samps_complex_first_d.csv',index=False)
            
            # raw_samps_.to_csv('pole_exp_1_clean/goldline/h5/samps_complex_first.csv',index=False)
            # raw_samps_.to_csv('pole_exp_1_clean/ten_yard/h5/samps_complex_first.csv',index=False)
            raw_samps_.to_csv('pole_exp_1_clean/twenty_yard/h5/samps_complex_first.csv',index=False)
            
            
            print('^^^^^^^^^^^^^^^^^^^^^ Raw #2 (complex) samples saved ')
            
        
            plt.plot(amplitude_value_csi) 
            
            # pole_exp_1_clean/twenty_yard/h10
            # pole_exp_1_clean/twenty_yard/h10
            # pole_exp_1_clean/twenty_yard/h10
            
            
            # goldline @ 5 
            # pole_exp_1/goldline/h5/first_d.hdf5
            # pole_exp_1/goldline/h5/first.hdf5
            # pole_exp_1/goldline/h5/sec_d.hdf5
            # pole_exp_1/goldline/h5/sec.hdf5
            # goldline @ 10
            # pole_exp_1/goldline/h10/first_d.hdf5
            # pole_exp_1/goldline/h10/first.hdf5
            # pole_exp_1/goldline/h10/sec_d.hdf5
            # pole_exp_1/goldline/h10/sec.hdf5
            # ##
            # 10y @ 5 
            # pole_exp_1/ten_yard/h5/first_d.hdf5
            # pole_exp_1/ten_yard/h5/first.hdf5
            # pole_exp_1/ten_yard/h5/sec_d.hdf5
            # pole_exp_1/ten_yard/h5/sec.hdf5
            # 10y @ 10
            # pole_exp_1/ten_yard/h10/first_d.hdf5
            # pole_exp_1/ten_yard/h10/first.hdf5
            # pole_exp_1/ten_yard/h10/sec_d.hdf5
            # pole_exp_1/ten_yard/h10/sec.hdf5
            # ##
            # 20y @ 5 
            # pole_exp_1/twenty_yard/h5/first_d.hdf5
            # pole_exp_1/twenty_yard/h5/first.hdf5
            # pole_exp_1/twenty_yard/h5/sec_d.hdf5
            # pole_exp_1/twenty_yard/h5/sec.hdf5
            # 20y @ 10
            # pole_exp_1/twenty_yard/h10/first_d.hdf5
            # pole_exp_1/twenty_yard/h10/first.hdf5
            # pole_exp_1/twenty_yard/h10/sec_d.hdf5
            # pole_exp_1/twenty_yard/h10/sec.hdf5
            print("########################")
            print('\n')
 
            
        # =============================================================================
        # End
        # =============================================================================
                    
            
            
        if analyze:
            analyze_hdf5(hdf5, ref_frame, ref_cell, ref_subcarrier, signal_offset)
scrpt_end = time.time()
print(">>>> Script Duration: time: %f \n" % ( scrpt_end - scrpt_strt) )
