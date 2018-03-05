#####################################################################
#                                                                   #
# PyCapture2_camera_server.py                                       #
#                                                                   #
# This file is part of the labscript suite (see                     #
# http://labscriptsuite.org) and is licensed under the Simplified   #
# BSD License. See the license.txt file in the root of the project  #
# for the full license.                                             #
#                                                                   #
# This camera server is an extension of camera_server.py.           #
# PyCapture2_camera_server implements a server and BLACS            #
# cameras using FLIR's python wrapper for FlyCapture2.              #
#                                                                   #
# To start the server:                                              #
# Run a command line in the directory containing this file          #
# and type:                                                         #
# python PyCapture2_camera_server.py <cameraNameFromBlacs>          #
# or type:                                                          #
# python PyCapture2_camera_server <cameraNameFromBlacs>             #
# <width> <height> <offsetX> <offsetY>                              #
# The optional inputs <width>, <height>, <offsetX>, and             #
# <offsetY> define an acquistion ROI for the camera.                #
# The default ROI is the full sensor.                               #
# Note that the acquisition ROI is overwritten on a per-shot        #
# basis if your BLACS camera's acquisition_ROI property is          #
# not None.                                                         #
#                                                                   #
# Author: dsbarker                                                  #
#                                                                   #
#####################################################################

# Future imports to ease change to Python 3.
# Only print_function is strictly necessary,
# but importing all to prevent issues with
# potential future features.
from __future__ import print_function
from __future__ import absolute_import
from __future__ import unicode_literals
from __future__ import division
import sys
import time
import Queue
import PyCapture2
import zprocess
from labscript_utils import check_version
import labscript_utils.shared_drive
# importing this wraps zlock calls around HDF file openings and closings:
import labscript_utils.h5_lock
from labscript_utils.camera_server import CameraServer
import h5py
import numpy as np
check_version('zprocess', '1.3.3', '3.0')

# Define the PyCap2_Camera class:
class PyCap2_Camera(object):

    def __init__(self, sn=None, alias=None, bus=None):

        # Get number of connected cameras:
        #numCams = bus.getNumOfCameras()
        #Try to connect to camera specified by sn:
        if bus is not None:
            if sn is not None:
                self.camera = PyCapture2.Camera()
                self.camera.connect(bus.getCameraFromSerialNumber(sn))
                print('Connected to' , alias)

                # Make sure a bunch of stuff is set correctly.
                self.camera.enableLUT(False) # Ensure no lookup table is used.
                # Turn off gamma:
                gamma = self.camera.getProperty(PyCapture2.PROPERTY_TYPE.GAMMA)
                gamma.onOff = False
                gamma.absControl = True
                gamma.absValue = 1
                self.camera.setProperty(gamma)
                # Turn off Auto Exposure
                autoExp = self.camera.getProperty(PyCapture2.PROPERTY_TYPE.AUTO_EXPOSURE)
                autoExp.onOff = True # For some reason, true holds the exposure value
                autoExp.absControl = True
                autoExp.autoManualMode = False
                autoExp.absValue = 0 # Not sure that this is best
                self.camera.setProperty(autoExp)
                # Set gain to 0 dB
                gain = self.camera.getProperty(PyCapture2.PROPERTY_TYPE.GAIN)
                gain.autoManualMode = False
                gain.absControl = True
                gain.absValue = 0
                self.camera.setProperty(gain)
                # Set sharpness for no image adjustment
                sharpness = self.camera.getProperty(PyCapture2.PROPERTY_TYPE.SHARPNESS)
                sharpness.onOff = False
                sharpness.autoManualMode = False
                sharpness.absValue = 1024 # 1024 corresponds to no adjustment
                self.camera.setProperty(sharpness)
                # Make sure framerate is not auto adjusting
                frameRate = self.camera.getProperty(PyCapture2.PROPERTY_TYPE.FRAME_RATE)
                frameRate.autoManualMode = False
                frameRate.absControl = True
                self.camera.setProperty(frameRate)
                # Make sure that shutter is not auto adjusting
                shutter = self.camera.getProperty(PyCapture2.PROPERTY_TYPE.SHUTTER)
                shutter.autoManualMode = False
                shutter.absControl = True
                self.camera.setProperty(shutter)
                # Set black level (a.k.a. Brightness)
                # THIS IS IMPORTANT FOR ATOM COUNTING!
                # Will need to calibrate for each camera.
                # There is a bug that prevents brightness from being set using setProperty.
                # Bug has been confirmed by FLIR, they probably are not going to fix it.
                # So, try using registers instead:
                bLevelReg = 0x800 # black level register is 800h
                bLevelVal = 0x8200000B # black level value is contained in last 2 digits.
                                       # Do not edit other digits!
                self.camera.writeRegister(bLevelReg,bLevelVal)
                bLevel = self.camera.getProperty(PyCapture2.PROPERTY_TYPE.BRIGHTNESS)
                print('Black Level = ', bLevel.absValue)

                # Set up the frame buffer:
                config = self.camera.getConfiguration()
                config.numBuffers = 10
                # May want to make the timeout configurable
                config.grabTimeout = 10000 #we'll overwrite this later
                config.grabMode = PyCapture2.GRAB_MODE.BUFFER_FRAMES
                # Do not set highPerformanceRetrieveBuffer true unless
                # the program has been thoroughly debugged.
                # So, NEVER.
                config.highPerformanceRetrieveBuffer = False
                self.camera.setConfiguration(config)

            else:
                print('Please specify a serial number for')
                print('this camera in the connection table.')
        else:
            print('Please specify a bus.')

    # disconnect from the physical camera:
    def disconnect(self):
        self.camera.disconnect()

    # set the trigger mode for the camera:
    def setTriggerMode(self,trigPolarity):

        trigMode = self.camera.getTriggerMode()
        trigMode.onOff = True
        trigMode.polarity = trigPolarity
        trigMode.source = 0 # Set for Flea3 dedicated digital input.
        trigMode.mode = 1 # Bulb trigger, seems most compatible
                          # with labscript camera class exposures.
        self.camera.setTriggerMode(trigMode)
        # Ensure there is no trigger delay
        trigDelay = self.camera.getTriggerDelay()
        trigDelay.onOff = False
        self.camera.setTriggerDelay(trigDelay)


    # Configure camera images
    # May want to add arguments at a later time for more flexibility.
    def setImageMode(self, width, height, offX, offY):

        # Check that format7, mode 0 is supported:
        fmt7Info, supported = self.camera.getFormat7Info(0)
        if supported:
            try:
                imageSettings, packetSize, percentage = self.camera.getFormat7Configuration()
                imageSettings.mode = 0 # set Format7, Mode 0
                # Set the ROI:
                # Round X offset down to multiple of offsetHStepSize:
                if offX > fmt7Info.offsetHStepSize:
                    imageSettings.offsetX = offX - (offX % fmt7Info.offsetHStepSize)
                else:
                    imageSettings.offsetX = 0
                # Round Y offset down to multiple of offsetVStepSize:
                if offY > fmt7Info.offsetVStepSize:
                    imageSettings.offsetY = offY - (offY % fmt7Info.offsetVStepSize)
                else:
                    imageSettings.offsetY = 0
                # Ensure width and height are valid given sensor size:
                if (width + imageSettings.offsetX) > fmt7Info.maxWidth:
                    width -= (width + imageSettings.offsetX) - fmt7Info.maxWidth
                if (height + imageSettings.offsetY) > fmt7Info.maxHeight:
                    height -= (height + imageSettings.offsetY) - fmt7Info.maxHeight
                if width < fmt7Info.maxWidth:
                    # Round image size down to multiple of imageHStepSize:
                    if width > fmt7Info.imageHStepSize:
                        imageSettings.width = width - (width % fmt7Info.imageHStepSize)
                    elif width == -1:
                        imageSettings.width = fmt7Info.maxWidth
                    else:
                        imageSettings.width = fmt7Info.imageHStepSize
                else:
                    imageSettings.width = fmt7Info.maxWidth
                if height < fmt7Info.maxHeight:
                    # Round image size down to multiple of imageVStepSize:
                    if height > fmt7Info.imageVStepSize:
                        imageSettings.height = height - (height % fmt7Info.imageVStepSize)
                    elif height == -1:
                        imageSettings.height = fmt7Info.maxHeight
                    else:
                        imageSettings.height = fmt7Info.imageVStepSize
                else:
                    imageSettings.height = fmt7Info.maxHeight
                print('(',imageSettings.width,',',imageSettings.height,',',imageSettings.offsetX,',',imageSettings.offsetY,')',sep="")
                # Set Mono16 as the image format.
                imageSettings.pixelFormat = PyCapture2.PIXEL_FORMAT.MONO16

                fmt7PktInf, valid = self.camera.validateFormat7Settings(imageSettings)
                if valid:
                    self.camera.setFormat7Configuration(100, imageSettings)
                    fmt7PktInf, valid = self.camera.validateFormat7Settings(imageSettings)
                    self.camera.setFormat7ConfigurationPacket(fmt7PktInf.maxBytesPerPacket, imageSettings)
            # Handle Errors
            except PyCapture2.Fc2error as fc2Err:
                print('Error getting image settings: ', fc2Err)
                print('Camera is not in Format7.')
        else:
            print('This camera does not support Format7, Mode 0.')
            print('Please use a different camera.')

    # grab number of images specified by n_images
    def grabMultiple(self, n_images, comm_queue):
        imgs = []
        try:
            self.camera.startCapture()
            print('Capture started.')
        except PyCapture2.Fc2error as fc2Err:
            print('Error starting capture: {e}'.format(e=fc2Err))
            print('Restart the server.')
            return imgs
        idx = 0
        while idx < n_images and comm_queue.empty(): # Check queue to catch a quit command.
            try:
                image = self.camera.retrieveBuffer()
                # If it is the first image, get its dimensions
                if idx==0:
                    n_Rows = image.getRows()
                    n_Cols = image.getCols()
                    print("n_Rows = {0:d}, n_Cols = {1:d}".format(n_Rows,n_Cols))
                # Immediately convert the image to a numpy array and store in
                # imgs list because we suspect that the image structure
                # returned by camera.retrieveBuffer() is not thread safe.
                imgs.append(np.array(image.getData(),dtype=np.uint8).view(np.uint16))
                if idx % 10 == 0:
                    print('retrieved {i}th image of {j} images'.format(i=idx+1,j=n_images))
            except PyCapture2.Fc2error as fc2Err:
                if idx == 0 and str(fc2Err) == '\'Timeout error.\'':
                    # Don't time out waiting for the first image.
                    idx -= 1
                else:
                    print('Error retrieving buffer: {e} at {i}th image.'.format(e=fc2Err,i=idx+1))
                    break
            idx += 1

        n_acq = len(imgs)
        print('got {a} images.'.format(a=n_acq))
        # In case of failure, return zeros for n_Rows and n_Cols.  In case of
        # success, resize each image into n_Rows and n_Cols
        if n_acq == 0:
            n_Rows = 0
            n_Cols = 0
        else:
            for img in imgs:
                img.resize(n_Rows,n_Cols)
        try:
            self.camera.stopCapture()
            print('Capture stopped.')
        except PyCapture2.Fc2error as fc2Err:
            print('Error stopping capture: {e}'.format(e=fc2Err))
            print('Restart the server.')
        return imgs, n_Rows, n_Cols

    # change timeout on the fly.
    def SetTimeout(self, max_wait=10.0):
        config = self.camera.getConfiguration()
        # Set grab timeout to be 1 second longer than max_wait
        config.grabTimeout = int(max_wait*1000 + 1000)
        self.camera.setConfiguration(config)

    # show PyCapture2 version:
    def printBuildInfo(self):
        libVer = PyCapture2.getLibraryVersion()
        print("PyCapture2 library version: ", libVer[0], libVer[1], libVer[2], libVer[3])
        print("\n")

    # show basic camera information:
    def printCameraInfo(self):
        camInfo = self.camera.getCameraInfo()
        print("\n*** CAMERA INFORMATION ***\n")
        print("Serial number - ", camInfo.serialNumber)
        print("Camera model - ", camInfo.modelName)
        print("Camera vendor - ", camInfo.vendorName)
        print("Sensor - ", camInfo.sensorInfo)
        print("Resolution - ", camInfo.sensorResolution)
        print("Firmware version - ", camInfo.firmwareVersion)
        print("Firmware build time - ", camInfo.firmwareBuildTime)
        print("\n")

    # show Format7 image capibilities:
    def printFormat7Capabilities(self, mode=0):
        fmt7info, supported = self.camera.getFormat7Info(mode)
        print("Max image pixels: ({}, {})".format(fmt7info.maxWidth, fmt7info.maxHeight))
        print("Image unit size: ({}, {})".format(fmt7info.imageHStepSize, fmt7info.imageVStepSize))
        print("Offset unit size: ({}, {})".format(fmt7info.offsetHStepSize, fmt7info.offsetVStepSize))
        print("Pixel format bitfield: 0x{}".format(fmt7info.pixelFormatBitField))
        print("\n")

class PyCap2_CameraServer(CameraServer):

    def __init__(self, port, camera_name, command_queue, results_queue):
        zprocess.ZMQServer.__init__(self, port, type='string')
        self._h5_filepath = None
        self.camera_name = camera_name
        self.command_queue = command_queue
        self.results_queue = results_queue
        self.imgs = []

    def transition_to_buffered(self, h5_filepath):
        # How many images to get
        with h5py.File(h5_filepath) as f:
            groupname = self.camera_name
            group = f['devices'][groupname]
            props = labscript_utils.properties.get(f, camera_name,'device_properties')
            if not 'EXPOSURES' in group:
                print('no camera exposures in this shot.')
                return
            n_images = len(group['EXPOSURES'])
            # Find max time between images:
            if n_images > 1:
                exp_times = f['devices'][groupname]['EXPOSURES']['time']
                max_wait = max(abs(x - y) for (x, y) in zip(exp_times[1:], exp_times[:-1]))
            else:
                max_wait = 0.1
            # Use acquisition_ROI property to set camera ROI:
            if 'acquisition_ROI' in props:
                if props['acquisition_ROI'] is not None:
                    width = props['acquisition_ROI'][0]
                    height = props['acquisition_ROI'][1]
                    offX = props['acquisition_ROI'][2]
                    offY = props['acquisition_ROI'][3]
                    print('resetting acquisition ROI')
                    # Tell acquisition mainloop to reset ROI:
                    self.command_queue.put(['set_ROI', (width, height, offX, offY)])
        print('max_wait = {} s'.format(max_wait))
        # Tell acquisition mainloop to reset timeout:
        self.command_queue.put(['set_timeout', max_wait])
        print('Configured for {n} images.'.format(n=n_images))
        # Tell the acquisition mainloop to get some images:
        self.command_queue.put(['acquire', n_images])

    def transition_to_static(self, h5_filepath):
        start_time = time.time()
        with h5py.File(h5_filepath) as f:
            groupname = self.camera_name
            group = f['devices'][groupname]
            if not 'EXPOSURES' in group:
                print('no camera exposures in this shot.')
                return
            n_images = len(group['EXPOSURES'])
            img_type = f['devices'][groupname]['EXPOSURES']['frametype']
            img_set = list(set(img_type))
            try:
                images, n_Rows, n_Cols = self.results_queue.get(timeout=1)
                if isinstance(images, Exception):
                    raise images
            except Queue.Empty:
                print('Timeout in image acquisition. Returning empty images.')
                images = []
            n_acq = len(images)
            print('Saving {a} images.'.format(a=n_acq))
            if images: # Only run save routine if images is non-empty
                # Create the group in which we will save the images:
                group = f.create_group('/images/' + f['devices'][groupname].attrs.get('orientation') + '/' + groupname)
                # Save images:
                imgs_toSave = {}
                for f_type in img_set:
                    imgs_toSave[f_type] = []
                    idx = 0
                    # all images should be same size:
                    for idx in range(n_acq):
                        if img_type[idx] == f_type:
                            imgs_toSave[f_type].append(images[idx])
                            # print('Creating dataset.')
                            group.create_dataset(f_type,data=np.array(imgs_toSave[f_type]))
                            """images_to_save = [imgs[idx] if img_type[idx] == f_type for idx in range(n_acq)]
                            group.create_dataset(f_type,data=np.array(images_to_save))"""
                            print(f_type +
                                ' camera shots saving time: {s} '.format(s=str(time.time()
                                -start_time))
                                + 's')

    def abort(self):
        # If abort gets called, probably need to break out of grabMultiple.
        # Put a dummy command into the queue to accomplish that.
        self.command_queue.put(['abort', None])

# acquisition_mainloop reads from a command queue and puts into a results queue.
def acquisition_mainloop(command_queue, results_queue, bus, camera_name, h5_attrs, width, height, offX, offY):
    # Start the camera:
    cam = PyCap2_Camera(sn=h5_attrs['serial_number'],alias=camera_name, bus=bus)
    # Print camera info for debugging:
    #cam.printCameraInfo()
    #cam.printFormat7Capabilities(0)
    # Set up the camera for image acquisition:
    cam.setImageMode(width, height, offX, offY)
    if h5_attrs['trigger_edge_type'] is 'rising': # Ensure that trigger polarity
                                                  # matches BLACS
        cam.setTriggerMode(1)
    else:
        cam.setTriggerMode(0)
    try:
        while True:
            command, args = command_queue.get()
            #print(command, args)
            if command == 'acquire':
                n_images = args
                try:
                    result = cam.grabMultiple(n_images, command_queue)
                except Exception as e:
                    result = e
            elif command == 'set_timeout':
                max_wait = args
                cam.SetTimeout(max_wait)
                continue # skip put into results_queue
            elif command == 'set_ROI':
                # Only perform ROI reset when necessary.
                if (width,height,offX,offY) != args:
                    width = args[0]
                    height = args[1]
                    offX = args[2]
                    offY = args[3]
                    cam.setImageMode(width, height, offX, offY)
                continue # skip put into results_queue
            elif command == 'abort':
                # command to cause grabMultiple to break
                results_queue.get(timeout=1) # Pull bad images out of queue.
                continue
            elif command == 'quit':
                break
            else:
                result = Exception('invalid command')
            results_queue.put(result)
    finally:
        cam.disconnect()

if __name__ == '__main__':
    # Import information about the lab configuration
    # and necessary modules:
    from labscript_utils.labconfig import LabConfig
    import labscript_utils.properties
    import h5py
    lc = LabConfig()
    import sys
    import threading
    # To start the server, type: "python PyCapture2_camera_server <camera_name>".
    # Check that a camera name is provided:
    try:
        camera_name = sys.argv[1]
    except IndexError:
        print('Call me with the name of a camera as defined in BLACS.')
        sys.exit(0)
    # Get the h5 path and camera properties.
    h5_filepath = lc.get('paths', 'connection_table_h5')
    with h5py.File(h5_filepath) as f:
        h5_attrs = labscript_utils.properties.get(f, camera_name,
                                                   'device_properties')
        h5_conn = labscript_utils.properties.get(f, camera_name,
                                                   'connection_table_properties')
    # Optionally, start the server using
    # "python PyCapture2_camera_server <camera_name> <width> <height> <offsetX> <offsetY>"
    # which allows you to define the camera ROI
    if len(sys.argv)-2 == 4:
        try:
            # set ROI parameters:
            width = int(sys.argv[2])
            height = int(sys.argv[3])
            offX = int(sys.argv[4])
            offY = int(sys.argv[5])
        except:
            raise
    else:
        # default to full sensor ROI:
        width = -1
        height = -1
        offX = -1
        offY = -1
    # Define synchronization Queues:
    command_queue = Queue.Queue()
    results_queue = Queue.Queue()
    # Start a camera bus and check for cameras:
    bus = PyCapture2.BusManager()
    numCams = bus.getNumOfCameras()
    if numCams > 0:
        print('starting camera server on port {port}...'.format(port=h5_conn['BIAS_port']))
        # Start the camera server:
        server = PyCap2_CameraServer(h5_conn['BIAS_port'], camera_name, command_queue, results_queue)
        # Start the acquisition loop:
        acquisition_thread = threading.Thread(target= acquisition_mainloop,
                                args=(command_queue, results_queue, bus, camera_name, h5_attrs, width, height, offX, offY))
        acquisition_thread.daemon=True
        acquisition_thread.start()
        server.shutdown_on_interrupt()
        command_queue.put(['quit', None])
        # The join should timeout so that infinite grab loops do not persist.
        acquisition_thread.join(1.0)
    else:
        print('No cameras detected.')
        sys.exit(0)
