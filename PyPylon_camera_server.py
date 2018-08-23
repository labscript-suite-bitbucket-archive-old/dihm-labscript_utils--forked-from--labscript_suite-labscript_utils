#####################################################################
#                                                                   #
# PyPylon_camera_server.py                                          #
#                                                                   #
# This file is part of the labscript suite (see                     #
# http://labscriptsuite.org) and is licensed under the Simplified   #
# BSD License. See the license.txt file in the root of the project  #
# for the full license.                                             #
#                                                                   #
# This camera server is an extension of camera_server.py.           #
# PyPylon_camera_server implements a server and BLACS               #
# cameras using the PyPylon python wrapper for Basler's Pylon SDK.  #
#                                                                   #
# To start the server:                                              #
# Run a command line in the directory containing this file          #
# and type:                                                         #
# python PyPylon_camera_server.py <cameraNameFromBlacs>             # 
# The default ROI is the full sensor.                               #
#                                                                   #
# Author: dihm                                                      #
#                                                                   #
#####################################################################

# Future imports to ease change to Python 3.
from __future__ import print_function, absolute_import, unicode_literals, division
import time
import pypylon
import numpy as np
import threading
import zprocess
#from labscript import LabscriptError
# cannot import from labscript without getting a scipy import in 
# optotunelens.py unit conversion
# scipy has bug that installs Fortran ctrl-C handler that overrides
# python's keyboard interrupt handling
from labscript_utils import check_version, PY2
import labscript_utils.shared_drive
import labscript_utils.h5_lock, h5py
from labscript_utils.camera_server import CameraServer
check_version('zprocess', '1.3.3', '3.0')

if PY2:
    import Queue
else:
    import queue as Queue

class PyPylon_Camera(object):

    def __init__(self, sn):
        '''Initialize camera with sn (as string).
        Set standard settings.'''

        cam_info = self.find_camera(sn)
        self.cam = pypylon.factory.create_device(cam_info)
        self.cam.open()
        print('Camera Created and Opened...')        

        # Ensure some default settings
        # Ensure no lookup table is used.
        self.cam.properties['LUTEnable'] = 'False' 
        # Turn off gamma:
        self.cam.properties['Gamma'] = 1.0
        # Turn off Auto Exposure & Gain
        self.cam.properties['ExposureAuto'] = 'Off'
        self.cam.properties['GainAuto'] = 'Off'
        # Set gain to 0 dB
        self.cam.properties['Gain'] = 0.0
        # Set black level (a.k.a. Brightness)
        self.cam.properties['BlackLevelSelector'] = 'All'
        self.cam.properties['BlackLevel'] = 0.0
        
        # save some parameters to the class for lookup in other functions
        self.maxX = self.cam.properties['WidthMax']
        self.maxY = self.cam.properties['HeightMax']

        self.width = self.cam.properties['Width']
        self.height = self.cam.properties['Height']
        self.offX = self.cam.properties['OffsetX']
        self.offY = self.cam.properties['OffsetY']
        
        # defualt to software triggers and rising edge activation
        self.setTriggerMode('Off')
        self.configureTrigger('rising')

        # these settings likely to become configurable later
        self.setFormat('Mono12')
        self.setROI(self.maxX,self.maxY,0,0)
        print('Camera configured....')
        
        # start the queues and acquisition thread
        self.command_queue = Queue.Queue()
        self.results_queue = Queue.Queue()

        self.acquisition_thread = threading.Thread(target=self.acquisition_mainloop,
                                                        args=())
        self.acquisition_thread.daemon = True
        self.acquisition_thread.start()
        print('Acquisiton Thread started....')

    def disconnect(self):
        print('Closing Camera....')
        self.cam.close()
        
    def reconnect(self):
        print('Reconnected to same Camera....')
        self.cam.open()
        
    def find_camera(self, sn):
        '''Find the camera with the specified serial number as string'''
        available_cams = pypylon.factory.find_devices()
        for cam_info in available_cams:
            if sn == cam_info.serial_number:
                return cam_info
               
        raise Exception('Camera {:s} not found!'.format(sn))
                
    def acquisition_mainloop(self):
        '''mainloop for acquisition
        Once camera is initialized, all interaction with hardware
        must go through this loop.'''
        try:
            while True:
                command, args = self.command_queue.get()
                if command == 'acquire':
                    n_images = args
                    try:
                        result = self.grabMultiple(n_images)
                    except Exception as e:
                        result = e
                elif command == 'set_exposure_time':
                    exp_time = args
                    self.setExposureTime(exp_time)
                    continue # skips put into results_queue
                elif command == 'configure_trigger':
                    polarity = args
                    self.configureTrigger(polarity)
                    continue
                elif command == 'set_trigger_mode':
                    mode = args
                    self.setTriggerMode(mode)
                    continue
                elif command == 'set_ROI':
                    if (self.width,self.height,self.offX,self.offY) != args: 
                        self.setROI(*args)
                    continue
                elif command == 'disconnect':
                    self.disconnect()
                    continue
                elif command == 'reconnect':
                    self.reconnect()
                    continue
                elif command == 'abort':
                    # clear out results queue when aborting
                    self.results_queue.get_nowait()
                elif command == 'quit':
                    break
                else:
                    result = Exception('invalid command')
                # send results to queue
                self.results_queue.put(result)
        finally:
            # shutdown camera when loop quits
            self.disconnect()

    def configureTrigger(self,trigPolarity):
        '''Setup camera for standard hardware trigger on Line1 source.
        Polarity is configurable: \'rising\' or \'falling\''''
        self.cam.properties['TriggerSelector'] = 'FrameStart'
        self.cam.properties['TriggerSource'] = 'Line1'
        if 'rising' in trigPolarity:
            pol = 'RisingEdge'
        elif 'falling' in trigPolarity:
            pol = 'FallingEdge'
        else:
            raise Exception('Unknown trigger type {}'.format(trigPolarity))
        self.cam.properties['TriggerActivation'] = pol
        
    def setTriggerMode(self,mode):
        '''Toggles trigger mode on and off.
        Options are \'On\' and \'Off\' '''
        self.cam.properties['TriggerMode'] = mode
        
    def setExposureTime(self,exp_time):
        '''Configures the camera exposure settings for timed exposure
        Exposure time is in seconds'''
        self.cam.properties['ExposureMode'] = 'Timed'
        self.cam.properties['ExposureTime'] = int(exp_time*1.0E6)
        
    def setFormat(self,pix_format):
        '''Sets the pixel format to allowed format
        Allowed formats: Mono8, Mono12
        Also configures the return numpy format.'''
        self.cam.properties['PixelFormat'] = pix_format
        pix_size = self.cam.properties['PixelSize']
        if pix_size.startswith('Bpp'):
            self.return_dtype = 'uint' + pix_size[3:]
        else:
            raise Exception('Pixel Size type {:s} not supported by PyPylon'.format(pix_size))

    def setROI(self, width, height, offX, offY):
        '''Sets ROI in pixels relative to sensor origin'''
        # confirm requested settings are valid for camera
        if (offX+width) > self.maxX:
            raise Exception('X offset:{}, width:{} > {}'.format(offX,width,self.maxX))
        if (offY+height) > self.maxY:
            raise Exception('Y offset:{}, height:{} > {}'.format(offY,height,self.maxY))
          
        # check requested against current settings
        currX = self.cam.properties['Width']
        currY = self.cam.properties['Height']
        currOffX = self.cam.properties['OffsetX']
        currOffY = self.cam.properties['OffsetY']
        
        if (currX == width) and (currY == height) and (offX == currOffX) and (offY == currOffY):
            # settings aren't changing
            pass
        else:
            # order matters when setting to avoid out of bounds errors
            if offX > self.maxX-currX:
                self.cam.properties['Width'] = width
                self.cam.properties['OffsetX'] = offX
            else:
                self.cam.properties['OffsetX'] = offX
                self.cam.properties['Width'] = width
            if offY > self.maxY-currY:
                self.cam.properties['Height'] = height
                self.properties['OffsetY'] = offY
            else:
                self.cam.properties['OffsetY'] = offY
                self.cam.properties['Height'] = height
                
        # finally, update expected array sizes for acquire array init
        self.width = width
        self.height = height
        # readout time depends on ROI
        print('ReadoutTime:',self.cam.properties['SensorReadoutTime']*1E-3,'ms')

    def grabMultiple(self, n_images):
        '''Grab multiple images from camera'''
        imgs = np.zeros((n_images,self.height,self.width),dtype=self.return_dtype)
        
        for i,image in enumerate(self.cam.grab_images(n_images)):
            if not self.command_queue.empty():
                print('Queue not empty during acquisition!')
                break
            imgs[i] = image
            print('{:d} of {:d} images acquired!'.format(i+1,n_images))
        return imgs

import click
import matplotlib.pyplot as plt
import matplotlib.animation as ani

class PyPylon_CameraServer(CameraServer):

    def __init__(self, port, camera_name, serial_number=''):
        zprocess.ZMQServer.__init__(self, port, type='string')
        self._h5_filepath = None
        self.camera_name = camera_name
        self.cam = PyPylon_Camera(sn=serial_number)
        
        # Start thread for detecting keyboard commands
        self.listener_thread = threading.Thread(target=self.connector,
                                                        args=())
        self.listener_thread.daemon = True
        self.listener_thread.start()
        print('Listener Thread started....')
        
        # start preview window
        self.fig = plt.figure(camera_name+' SN:'+serial_number)
        self.ax = plt.subplot(1,1,1)
        # preload with a dummy image
        first_preview = self.grab_image()
        self.preview_dims = first_preview.shape
        self.preview = self.ax.imshow(first_preview)
        # callback funtion to update preview plot, update interval is in ms
        animate = ani.FuncAnimation(self.fig,self.grab_preview,interval=500)
        
        self.close_preview = False
        self.run_preview = False
        self.running_shot = False
        self.cam_open = True
        plt.show()
            
    def connector(self):
        '''Listens for single character intput into command window.
        c-->closes access to camera
        r-->reconnects to camera
        p-->starts preview
        s-->stops preview'''
        try:
            while True:
                c = click.getchar()
                if c == 'c' and self.cam_open:
                    self.cam.command_queue.put(['disconnect',None])
                    self.cam_open = False
                    continue
                if c == 'r' and not (self.cam_open or self.running_shot):
                    self.cam.command_queue.put(['reconnect',None])
                    self.cam_open = True
                    continue
                if c == 'p' and self.cam_open and not (self.run_preview or self.running_shot):
                    self.cam.command_queue.put(['set_trigger_mode','Off'])
                    self.run_preview = True
                    continue
                if c == 's' and self.run_preview:
                    self.run_preview = False
                    self.cam.command_queue.put(['set_trigger_mode','On'])
                    print('Preview Stopped')
                    continue
        except KeyboardInterrupt:
            self.close_preview = True
        finally:
            print('Listener stopped.')

    def transition_to_buffered(self, h5_filepath):
        '''Configures acquisitions based on h5_file properties and exp table'''
        # need to make sure camera is open and not running previews
        if self.run_preview:
            self.run_preview = False
            self.cam.command_queue.put(['set_trigger_mode','Off'])
        if not self.cam_open:
            self.cam.command_queue.put(['reconnect',None])
            self.cam_open = True
        self.running_shot = True
            
        with h5py.File(h5_filepath) as f:
            groupname = self.camera_name
            group = f['devices'][groupname]
            props = labscript_utils.properties.get(f, camera_name,'device_properties')
            if not 'EXPOSURES' in group:
                print('no camera exposures in this shot.')
                return
            
            n_images = len(group['EXPOSURES'])
            if 'acquisition_ROI' in props:
                if props['acquisition_ROI'] is not None:
                    print('resetting acquisition ROI')
                    self.cam.command_queue.put(['set_ROI', tuple(props['acquisition_ROI'])])
            if 'exposure_time' in props:
                if props['exposure_time'] is not float('nan'):
                    print('Configuring Exposure Time....')
                    self.cam.command_queue.put(['set_exposure_time',props['exposure_time']])
            if 'trigger_edge_type' in props:
                print('Configuring Trigger Mode....')
                self.cam.command_queue.put(['configure_trigger',props['trigger_edge_type']])
                
        print('Configured for {n} image(s).'.format(n=n_images))
        # Tell the acquisition mainloop to get some images:
        self.cam.command_queue.put(['acquire', n_images])

    def transition_to_static(self, h5_filepath):
        '''Saves images from results queue, if there were exposures
        Expects images from queue to be numpy array, (n_images,height,width)'''
        start_time = time.time()
        with h5py.File(h5_filepath) as f:
            groupname = self.camera_name
            group = f['devices'][groupname]
            if not 'EXPOSURES' in group:
                print('no camera exposures in this shot.')
                return
            
            n_images = len(group['EXPOSURES'])
            img_type = f['devices'][groupname]['EXPOSURES']['frametype']
            # get distinct frametypes
            img_set = np.unique(img_type)
            try:
                images = self.cam.results_queue.get(timeout=1)
                if isinstance(images, Exception):
                    raise images
            except Queue.Empty:
                print('Timeout in image acquisition.')
                return
                
            n_acq = len(images)
            print('Saving {a} images.'.format(a=n_acq))
            # Create the group in which we will save the images:
            group = f.create_group('/images/' + f['devices'][groupname].attrs.get('orientation') + '/' + groupname)
            # Save images:
            for f_type in img_set:
                mask = img_type == f_type
                if sum(mask) == 1:
                    # only one image for this f_type, save as 2-D array
                    save_imgs = images[mask][0]
                else:
                    # save as 3-D array, (n_images,height,width)
                    save_imgs = images[mask]
                group.create_dataset(f_type,data=save_imgs)
                print(f_type,'camera shots saving time: {:.5f}'.format(time.time()-start_time),'s')
        
        # unblock preview mode
        self.running_shot = False
                
    def grab_image(self):
        '''Gets single image from camera'''
        self.cam.command_queue.put(['acquire',1])
        try:
            images = self.cam.results_queue.get(timeout=1)
            if isinstance(images, Exception):
                raise images
        except Queue.Empty:
            print('Timeout in image acquisition.')
            return np.zeros(self.preview_dims)
        
        return images[0]
        
    def grab_preview(self,i):
        '''Gets a preview and updates if self.run_preview = True'''
        if self.run_preview:
        
            self.preview.set_data(self.grab_image())
            
        if self.close_preview:
            plt.close('all')

    def abort(self):
        '''If BLACS calls abort, ensure grabMultiple exits'''
        self.cam.command_queue.put(['abort', None])

if __name__ == '__main__':
    # Import information about the lab configuration
    # and necessary modules:
    from labscript_utils.labconfig import LabConfig
    import labscript_utils.properties
    import labscript_utils.h5_lock, h5py
    lc = LabConfig()
    import sys
    # To start the server, type: "python PyPylon_camera_server <camera_name>".
    try:
        camera_name = sys.argv[1]
    except IndexError:
        raise Exception('Call me with the name of a camera as defined in BLACS.')
    # Get the h5 path and camera properties.
    h5_filepath = lc.get('paths', 'connection_table_h5')
    with h5py.File(h5_filepath,'r') as f:
        h5_attrs = labscript_utils.properties.get(f, camera_name,
                                                   'device_properties')
        h5_conn = labscript_utils.properties.get(f, camera_name,
                                                 'connection_table_properties')

    try:
        print('starting camera server on port {port}...'.format(port=h5_conn['BIAS_port']))
        print('Camera serial number {}'.format(h5_attrs['serial_number']))
        # Start the camera server:
        server = PyPylon_CameraServer(h5_conn['BIAS_port'], camera_name, str(h5_attrs['serial_number']))
        server.shutdown_on_interrupt()
        server.cam.command_queue.put(['quit', None])
        # The join should timeout so that infinite grab loops do not persist.
        server.cam.acquisition_thread.join(10.0)
        server.listener_thread.join(10.0)
    except:
        raise
    else:
        sys.exit(0)
