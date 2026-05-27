import os
import numpy as np
from datetime import datetime
from PIL import Image, ImageTk
import tkinter as tk
from tkinter import filedialog, messagebox

from pycromanager import Core

import time
import serial

import cv2
import sys
from ctypes import *

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from SkyraController import SkyraController

# Add path to the directory where the DLLs are stored for the scientific compact camera
os.add_dll_directory(r"C:\Users\macka\Downloads\Scientific Camera Interfaces\SDK\Native Toolkit\dlls\Native_64_lib")

from thorlabs_tsi_sdk.tl_camera import TLCameraSDK, OPERATION_MODE
from ThorlabsCamera import ThorlabsCamera

class TaggedImage:

    """
    Helper class to define an image
    """

    def __init__(self, frame, camera):

        """
        Initialization of an image object
        """

        self.pix = frame.image_buffer
        self.tags = {
            "Height": camera.image_height_pixels,
            "Width": camera.image_width_pixels,
        }

class MicroscopeGUI:

    """
    Creates a MicroscopeGUI object and handles all functionality related to the microscope
    """

    def __init__(self, root):

        """
        Initialization of the microscope
        Includes initialization of the control window, the camera, the motorized stages, the beam blocker, the laser, and
        all variables
        """

        self.root = root
        self.root.state("zoomed")
        self.root.title("Microscope Control")

        # Camera initialization
        try:
            from windows_setup import configure_path
            configure_path()
        except ImportError:
            configure_path = None

        self.sdk = TLCameraSDK()

        available_cameras = self.sdk.discover_available_cameras()
        if len(available_cameras) < 1:
            print("No cameras detected")
            raise RuntimeError("No Thorlabs camera detected")
        
        self.camera = self.sdk.open_camera(available_cameras[0])

        # Set initial camera settings
        self.camera.exposure_time_us = 100000
        self.camera.frames_per_trigger_zero_for_unlimited = 0
        self.camera.image_poll_timeout_ms = 1000
        self.camera.frame_rate_control_value = 10
        self.camera.is_frame_rate_control_enabled = True

        self.camera.arm(2)
        self.camera.issue_software_trigger()

        self.live_running = False # Variable for live imaging

        # -----------------------------------------------------------------------------------------------------------------------------

        # Arduino initialization

        # Replace COM5 with your Arduino COM port
        self.arduino = serial.Serial('COM5', 9600, timeout=1)
        time.sleep(2)  # Let Arduino reset

        # -----------------------------------------------------------------------------------------------------------------------------

        # Laser initialization

        self.skyra = SkyraController(port="COM7")

        self.power_var = tk.StringVar(value="0") # Laser power variable

        # -----------------------------------------------------------------------------------------------------------------------------

        # KDC101 DLL setup (motorized stages)
        if sys.version_info < (3, 8):
            os.chdir(r"C:\Program Files\Thorlabs\Kinesis") # Directory for DLLs for KDC101
        else:
            os.add_dll_directory(r"C:\Program Files\Thorlabs\Kinesis") # Directory for DLLs for KDC101

        self.lib = cdll.LoadLibrary("Thorlabs.MotionControl.KCube.DCServo.dll")

        # KDC101 serial numbers
        self.serial_1 = c_char_p(b"27269838") # Sample
        self.serial_2 = c_char_p(b"27269864") # Camera

        # KDC101 initialization
        # Build device list and initialize motors
        if self.lib.TLI_BuildDeviceList() != 0:
            raise RuntimeError("No KDC devices found")

        self.open_and_initialize(self.serial_1)
        self.open_and_initialize(self.serial_2)

        print("Motors homed and ready")

        # -----------------------------------------------------------------------------------------------------------------------------

        # Variables

        # Positioning and z-stack acquisition
        self.sample_position = 0.0          # Initial sample stage position
        self.camera_position = 0.0          # Initial camera stage position
        self.start_position_sample = 5.9    # Variable for starting z-stack acquisition for sample stage
        self.start_position_camera = 13.84  # Variable for starting z-stack acquisition for camera stage
        self.step_size = tk.StringVar()
        self.number_of_images = tk.StringVar()
        self.sample_positions = []
        self.camera_positions = []
        
        # Camera
        self.save_directory = "C:/Users/macka/Pictures/Thorlabs" # Initial directory for saving images
        self.live_running = False
        self.live_job = None
        self.image_label = tk.Label(self.root, bg="black")

        self.zoom_factor = 1.0
        self.display_min = 0
        self.display_max = 65535
        self.current_min = 0
        self.current_max = 65535
        self.auto_stretch_min = 0
        self.auto_stretch_max = 65535
        self.initial_display_min = 0
        self.initial_display_max = 65535
        self.last_frame = None

        self.pan_x = None
        self.pan_y = None
        self.dragging = False
        self.drag_start_x = 0
        self.drag_start_y = 0

        self.auto_stretch = True
        self.updating_sliders = False

        self.pixel_mode = False

        # Beam blocker
        self.beam_blocker = False

        # Laser
        self.laser_var = False

        self.laser_vars = {
            405: tk.IntVar(value=0),
            488: tk.IntVar(value=0),
            561: tk.IntVar(value=0),
            638: tk.IntVar(value=0),
        }

        # Calibration coefficient
        self.cal_coeff = 0.337500 # 0.333338

        # Measurement with image quality analysis
        self.iqa_measurement_var = False
        self.counter_within_range = 0
        self.counter_out_of_range = 0

        # Fast version of measurement with image quality analysis
        self.iqa_fast_measurement_var = False
        self.image_quality_var = 0.0
        self.counter_no_calibration = 0
        self.counter_calibration = 0
        self.threshold_percentage = 0.99

        # Initial selected image quality analyzer
        self.mode_var = tk.StringVar(value="Brenner gradient")

        # Safe close handler
        self.root.protocol("WM_DELETE_WINDOW", self.safe_shutdown)

        # Main container
        self.container = tk.Frame(root)
        self.container.pack(fill="both", expand=True)

        self.container.grid_rowconfigure(1, weight=1)
        self.container.grid_columnconfigure(0, weight=4)
        self.container.grid_columnconfigure(1, weight=1)

        self.build_menu()

    # -----------------------------------------------------------------------------------------------------------------------------

    # Helper methods to KDC101 (motorized stages)

    def get_position_mm(self, serial):

        """
        Get the position of a KDC101 based of the serial number
        The position of the KDC101 is returned in the form of a double showing the position in millimeters
        """

        dev_pos = c_int()
        real_pos = c_double()

        self.lib.CC_RequestPosition(serial)
        time.sleep(0.02)
        dev_pos.value = self.lib.CC_GetPosition(serial)

        self.lib.CC_GetRealValueFromDeviceUnit(
            serial, dev_pos, byref(real_pos), 0
        )
        return real_pos.value

    def wait_until_position(self, serial, target_mm, tol=0.001):

        """
        Waits until the movement of the KDC101 has been performed
        """

        while True:
            pos = self.get_position_mm(serial)
            if abs(pos - target_mm) <= tol:
                break
            time.sleep(0.05)

    def move_absolute_mm(self, serial, target_mm):

        """
        Moves a KDC101 to a specified position
        """

        dev_units = c_int()
        self.lib.CC_GetDeviceUnitFromRealValue(
            serial, c_double(target_mm), byref(dev_units), 0
        )
        self.lib.CC_SetMoveAbsolutePosition(serial, dev_units)
        self.lib.CC_MoveAbsolute(serial)
        self.wait_until_position(serial, target_mm)

    def open_and_initialize(self, serial):

        """
        Initialization of the KDC101
        """

        self.lib.CC_Open(serial)

        # Standard KDC101 Settings
        STEPS_PER_REV = c_double(1919.64186)
        GEARBOX_RATIO = c_double(1.0)
        PITCH = c_double(0.05555)

        self.lib.CC_SetMotorParamsExt(
            serial, STEPS_PER_REV, GEARBOX_RATIO, PITCH
        )

        self.lib.CC_StartPolling(serial, c_int(200))
        time.sleep(0.5)
        self.lib.CC_ClearMessageQueue(serial)

        # Homing
        zero = c_double(0.0)
        zero_dev = c_int()
        self.lib.CC_GetDeviceUnitFromRealValue(serial, zero, byref(zero_dev), 0)
        self.lib.CC_SetMoveAbsolutePosition(serial, zero_dev)
        self.lib.CC_MoveAbsolute(serial)
        self.wait_until_position(serial, 0.0)

        self.lib.CC_EnableChannel(serial)
        time.sleep(0.2)

    # -----------------------------------------------------------------------------------------------------------------------------

    # -----------------------------------------------------------------------------------------------------------------------------
    # Main Menu
    # -----------------------------------------------------------------------------------------------------------------------------

    def build_menu(self):

        """
        Builds the main menu of the program
        """

        self.clear_container()

        tk.Label(self.container, text="Main Menu",
                 font=("Arial", 16)).pack(pady=20)

        tk.Button(self.container, text="Calibration",
                  command=self.enter_calibration).pack(pady=10)

        tk.Button(self.container, text="Measurement",
                  command=self.enter_measurement).pack(pady=10)
        
    # -----------------------------------------------------------------------------------------------------------------------------

    # -----------------------------------------------------------------------------------------------------------------------------
    # Calibration
    # -----------------------------------------------------------------------------------------------------------------------------

    def enter_calibration(self):

        """
        Enter calibration mode
        Creates the window where the user can capture images, calculate the calibration coefficient of the camera step size,
        and find the starting positions of the camera
        """

        self.clear_container()

        tk.Label(self.container, text="Calibration Mode",
                 font=("Arial", 14)).grid(row=0, column=0, columnspan=2, pady=10) #pack(pady=10)

        # -----------------------------------------------------------------------------------------------------------------------------

        # Left side
        # Live view of the camera

        self.left_frame = tk.Frame(self.container, bg="black")
        self.left_frame.grid(row=1, column=0, sticky="nsew")

        self.image_label = tk.Label(self.left_frame, bg="black")
        self.image_label.pack_propagate(False)
        self.image_label.pack(fill="both", expand=True)

        # Mouse bindings
        self.image_label.bind("<MouseWheel>", self.mouse_zoom)
        self.image_label.bind("<ButtonPress-1>", self.start_drag)
        self.image_label.bind("<B1-Motion>", self.drag_image)
        self.image_label.bind("<ButtonRelease-1>", self.stop_drag)

        # -----------------------------------------------------------------------------------------------------------------------------

        # Right side 
        # Controls

        self.right_frame = tk.Frame(self.container)
        self.right_frame.grid(row=1, column=1, sticky="ns")

        # Scrollbar
        canvas = tk.Canvas(self.right_frame, width=480)
        scrollbar = tk.Scrollbar(self.right_frame, orient="vertical", command=canvas.yview)

        self.scrollable_frame = tk.Frame(canvas)

        self.scrollable_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))

        canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")

        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Labeled frames
        self.live_controls = tk.LabelFrame(self.scrollable_frame, text="Camera", padx=5, pady=5)
        self.live_controls.pack(fill="x", pady=(5,10))

        self.calibration_controls = tk.LabelFrame(self.scrollable_frame, text="Motors", padx=5, pady=5)
        self.calibration_controls.pack(fill="x", pady=(5, 10))

        self.beam_blocker_controls = tk.LabelFrame(self.scrollable_frame, text="Beam Blocker", padx=5, pady=5)
        self.beam_blocker_controls.pack(fill="x", pady=(5, 10))

        self.laser_controls = tk.LabelFrame(self.scrollable_frame, text="Laser", padx=5, pady=5)
        self.laser_controls.pack(fill="x", pady=(5, 10))

        self.other_controls = tk.LabelFrame(self.scrollable_frame, text="Saving Directory and Exiting", padx=5, pady=5)
        self.other_controls.pack(fill="x")

        # -----------------------------------------------------------------------------------------------------------------------------
        
        # Camera frame

        live_imaging_frame = tk.LabelFrame(self.live_controls, text="Live Imaging")
        live_imaging_frame.pack(fill="x", pady=5)

        # Exposure
        tk.Label(live_imaging_frame, text="Exposure (ms)").grid(row=0, column=0, padx=5, pady=(10,5))

        self.exposure_var = tk.StringVar()
        exposure_ms = self.camera.exposure_time_us / 1000
        self.exposure_var.set(str(exposure_ms))

        self.exposure_entry = tk.Entry(live_imaging_frame, textvariable=self.exposure_var, width=47)
        self.exposure_entry.grid(row=0, column=1, padx=5, pady=(10,5))
        tk.Button(live_imaging_frame, text="Set Exposure", command=self.set_exposure_from_entry).grid(row=0, column=2, padx=5, pady=(10,5)) #.pack(pady=3)

        # 1:1 pixel mode
        self.pixel_button = tk.Button(live_imaging_frame, text="1:1 Pixels", command=self.toggle_pixel_mode)
        self.pixel_button.grid(row=1, column=1, padx=5, pady=(5,5))

        # Autostretch
        self.auto_stretch_button = tk.Button(live_imaging_frame, text="Auto-Stretch: ON", command=self.toggle_autostretch)
        self.auto_stretch_button.grid(row=2, column=1, padx=5, pady=(5,10))

        # Brightness and Contrast Label Frame 
        self.b_c_labelframe = tk.LabelFrame(self.live_controls, text="Brightness and Contrast")
        self.b_c_labelframe.pack(fill="x", padx=5, pady=5)

        MAXVAL = 65535

        self.min_slider = tk.Scale(
            self.b_c_labelframe, 
            from_=0,
            to=MAXVAL,
            orient="horizontal",
            label="Minimum",
            command=self.min_change
            )
        self.min_slider.pack(fill="x")

        self.max_slider = tk.Scale(
            self.b_c_labelframe, 
            from_=0,
            to=MAXVAL,
            orient="horizontal",
            label="Maximal",
            command=self.max_change
            )
        self.max_slider.set(MAXVAL)
        self.max_slider.pack(fill="x")

        self.brightness_slider = tk.Scale(
            self.b_c_labelframe, 
            from_=-1.0,
            to=1.0,
            resolution=0.01, 
            orient="horizontal",
            label="Brightness",
            command=self.brightness_change
            )
        self.brightness_slider.pack(fill="x")

        self.contrast_slider = tk.Scale(
            self.b_c_labelframe, 
            from_=0.1,
            to=3.0,
            resolution=0.01,
            orient="horizontal",
            label="Contrast",
            command=self.contrast_change
            )
        self.contrast_slider.set(1.0)
        self.contrast_slider.pack(fill="x")

        self.button_frame = tk.Frame(self.b_c_labelframe)
        self.button_frame.pack(pady=5)

        tk.Button(self.button_frame, text="Auto", command=self.auto_contrast).pack(side="left", padx=5)
        tk.Button(self.button_frame, text="Reset", command=self.reset_contrast).pack(side="left", padx=5)
        tk.Button(self.button_frame, text="Set Min-Max", command=self.set_min_max).pack(side="left", padx=5)
        
        # Capture image and calibration frame
        capture_image_frame = tk.LabelFrame(self.live_controls, text="Capture Image and Calibration")
        capture_image_frame.pack(fill="x", pady=5)

        tk.Button(capture_image_frame, text="Capture Image",
                  command=self.capture_calibration_image).pack(pady=5)
        
        tk.Button(capture_image_frame, text="Capture Image using Image Quality Analysis",
                  command=self.capture_image_with_iqa).pack(pady=5)

        # Frame for image quality analyzers
        image_analysis_options = ["Brenner gradient", "Fast FT", "Laplacian variance", "Tenengrad"]
        frame = tk.LabelFrame(capture_image_frame, text="Image Quality Analyzers")
        frame.pack(fill="x")

        for i, opt in enumerate(image_analysis_options):
            tk.Radiobutton(
                frame,
                text=opt,
                variable=self.mode_var,
                value=opt
            ).grid(row=0, column=i, padx=5)

        for i in range(len(image_analysis_options)):
            frame.grid_columnconfigure(i, weight=1)

        # -----------------------------------------------------------------------------------------------------------------------------

        # Motors frame

        # Sample position
        tk.Label(self.calibration_controls, text="Sample Position (mm) - {0-23 mm}").grid(row=0, column=0, sticky="w", padx=5, pady=(10,5))
        
        self.sample_position_var = tk.StringVar()
        self.sample_position_var.set(str(self.get_position_mm(self.serial_1)))
        self.sample_position_entry = tk.Entry(self.calibration_controls, textvariable=self.sample_position_var, width=25)
        self.sample_position_entry.grid(row=0, column=1, padx=5, pady=(10,5))

        tk.Button(self.calibration_controls, 
                  text="Move Sample", 
                  command=self.move_sample_from_entry).grid(row=0, column=2, padx=5, pady=(10,5))

        # Camera position
        tk.Label(self.calibration_controls, text="Camera Position (mm) - {0-23 mm}").grid(row=1, column=0, padx=5, pady=(5,10))
        
        self.camera_position_var = tk.StringVar()
        self.camera_position_var.set(str(self.get_position_mm(self.serial_2)))
        self.camera_position_entry = tk.Entry(self.calibration_controls, textvariable=self.camera_position_var, width=25)
        self.camera_position_entry.grid(row=1, column=1, padx=5, pady=(5,10))

        tk.Button(self.calibration_controls, 
                  text="Move Camera", 
                  command=self.move_camera_from_entry).grid(row=1, column=2, padx=5, pady=(5,10))
        
        # Buttons
        tk.Button(self.calibration_controls, 
                  text="Find Best Camera Position with Image Quality Analysis",
                  command=self.find_best_camera_position).grid(row=2, column=0, columnspan=3, pady=5)

        tk.Button(self.calibration_controls, 
                  text="Use Current Positions for Calibration",
                  command=self.use_for_calibration).grid(row=3, column=0, columnspan=3, pady=5)
        
        tk.Button(self.calibration_controls, text="Show Calibration Measurements", 
                  command=self.show_calibration_plots).grid(row=4, column=0, columnspan=3, pady=5)
        
        # -----------------------------------------------------------------------------------------------------------------------------
        
        # Beam blocker frame

        self.beam_blocker_button = tk.Button(self.beam_blocker_controls, text="Beam Blocker Status: BLOCKING",
                  command=self.toggle_beam_blocker)
        self.beam_blocker_button.pack(pady=10)

        # -----------------------------------------------------------------------------------------------------------------------------

        # Laser settings frame

        # Illumination wavelength buttons
        for i, wl in enumerate([405, 488, 561, 638]):

            tk.Checkbutton(
                self.laser_controls,
                text=f"{wl} nm",
                variable=self.laser_vars[wl],
                command=lambda w=wl: self.select_laser(w)
            ).grid(row=1, column=i, padx=5, pady=(5,5))

        for i in range(len(self.laser_vars)):
            self.laser_controls.grid_columnconfigure(i, weight=1)

        # Set power of selected wavelength(s)
        tk.Label(self.laser_controls, text="Power (mW)").grid(row=2, column=0, padx=5, pady=(5,5))

        tk.Entry(self.laser_controls, textvariable=self.power_var, width=50).grid(row=2, column=1, columnspan=2, padx=5, pady=(5,5))

        tk.Button(self.laser_controls, text="Set Power", command=self.set_laser_power).grid(row=2, column=3, padx=5, pady=(5,5))

        # Toggle emission
        self.toggle_laser_button = tk.Button(self.laser_controls, text="Laser Status: OFF",
                  command=self.toggle_laser)
        self.toggle_laser_button.grid(row=3, column=1, columnspan=2, padx=5, pady=(15,10))

        # -----------------------------------------------------------------------------------------------------------------------------

        # Save and exiting frame
        tk.Button(self.other_controls, text="Save Current Live View as an Image", 
                  command=self.save_image).pack(pady=5)

        tk.Button(self.other_controls, text="Select Save Directory",
                  command=self.select_directory).pack(pady=5)
        
        tk.Button(self.other_controls, text="Back to Menu",
                  command=self.exit_calibration).pack(pady=10)
        # -----------------------------------------------------------------------------------------------------------------------------

        # Start live imaging
        self.start_live()

    def start_live(self): 

        """
        Starts live imaging mode of the camera Thorlabs CS126MU
        """

        if self.live_running:
            return

        print("Starting live mode")

        try:
            self.camera.disarm()
        except:
            pass

        self.camera.roi = (
            0,
            0,
            self.camera.sensor_width_pixels,
            self.camera.sensor_height_pixels
        )

        self.camera.arm(2)
        self.camera.issue_software_trigger()
        self.live_running = True
        self.update_live()

    def update_live(self):

        """
        Updates the imaging frame that is live imaging from the image that the camera is acquiring
        A copy of the image that the camera is acquiring is sent to be shown in the program
        """

        if not self.live_running:
            return

        try:
            frame = self.camera.get_pending_frame_or_null()
            if frame is None:
                self.live_job = self.root.after(30, self.update_live)
                return
            
            if frame.image_buffer is None:
                self.live_job = self.root.after(30, self.update_live)
                return
            
            tagged = TaggedImage(frame, self.camera)

            pixels = np.reshape(
                tagged.pix,
                (tagged.tags['Height'], tagged.tags['Width'])
            )

            self.last_frame = pixels.copy()

            self.render_image(pixels)
        except Exception as e:
            print("Live Error:", e)

        self.live_job = self.root.after(30, self.update_live)

    def render_image(self, pixels):
        
        """
        Renders the image seen by the camera by manipulating a copy of the camera's image acquisition
        """

        img = pixels.astype(np.float32)
        
        # auto-stretch
        if self.auto_stretch:
            self.auto_stretch_min = float(np.percentile(img, 0.5))
            self.auto_stretch_max = float(np.percentile(img, 99.8))

            min_val = self.auto_stretch_min
            max_val = self.auto_stretch_max
        else:
            min_val = self.display_min
            max_val = self.display_max

        if max_val <= min_val:
            return
        
        img = np.clip(img, min_val, max_val)
        img = (img - min_val) / (max_val - min_val)
        img *= 255
        img = img.astype(np.uint8)

        h, w = img.shape

        if self.pan_x is None or self.pan_y is None:
            self.pan_x = w // 2
            self.pan_y = h // 2

        # zoom
        if self.zoom_factor > 1.0:
            crop_w = int(w / self.zoom_factor)
            crop_h = int(h / self.zoom_factor)

            left = int(self.pan_x - crop_w // 2)
            left = max(0, min(left, w - crop_w))
            right = left + crop_w

            top = int(self.pan_y - crop_h // 2)
            top = max(0, min(top, h - crop_h))
            bottom = top + crop_h

            img = img[top:bottom, left:right]

        display_h, display_w = img.shape

        if not self.image_label.winfo_ismapped():
            label_w, label_h = 640, 480
        else:
            label_w = self.image_label.winfo_width()
            label_h = self.image_label.winfo_height()

        img_pil = Image.fromarray(img)
        
        # 1:1 pixel mode
        if self.pixel_mode:
            crop_w = min(label_w, display_w)
            crop_h = min(label_h, display_h)

            left = int(self.pan_x - crop_w // 2)
            top = int(self.pan_y - crop_h // 2)

            left = max(0, min(left, display_w - crop_w))
            top = max(0, min(top, display_h - crop_h))

            right = left + crop_w
            bottom = top + crop_h

            img_pil = img_pil.crop((left, top, right, bottom))
        else:
            scale = min(label_w / display_w, label_h / display_h)

            new_w = int(display_w * scale)
            new_h = int(display_h * scale)

            img_pil = img_pil.resize((new_w, new_h), Image.NEAREST)

        imgtk = ImageTk.PhotoImage(img_pil)
        self.image_label.imgtk = imgtk
        self.image_label.configure(image=imgtk)

    def mouse_zoom(self, event):

        """
        Listens to mouse scroll to zoom and zoom out in the rendered image
        The centered position is found by finding the position of the mouse in the rendered image area
        """

        if self.last_frame is None:
            return
        
        if event.delta > 0:
            self.zoom_factor *= 1.2
        else:
            self.zoom_factor /= 1.2

        self.zoom_factor = max(1.0, min(self.zoom_factor, 15))
        zoom_new = self.zoom_factor

        img_x_before, img_y_before = self.screen_to_image_coords(event.x, event.y)

        label_w = self.image_label.winfo_width()
        label_h = self.image_label.winfo_height()

        self.pan_x = img_x_before - (event.x - label_w / 2) / zoom_new
        self.pan_y = img_y_before - (event.y - label_h / 2) / zoom_new

        self.render_image(self.last_frame)

    def screen_to_image_coords(self, sx, sy):

        """
        Finds the coordinates of the mouse position
        """

        h, w = self.last_frame.shape

        label_w = max(1, self.image_label.winfo_width())
        label_h = max(1, self.image_label.winfo_height())

        if self.pixel_mode:
            img_x = self.pan_x + (sx - label_w / 2) / self.zoom_factor
            img_y = self.pan_y + (sy - label_h / 2) / self.zoom_factor
        else:
            img_x = sx / label_w * w
            img_y = sy / label_h * h

        return img_x, img_y

    def stop_live(self):

        """
        Stops live imaging
        """

        if not self.live_running:
            return
        
        print("Stopping live mode")
        self.live_running = False

        if self.live_job is not None:
            self.root.after_cancel(self.live_job)
            self.live_job = None

        self.camera.disarm()
        self.camera.dispose()
        self.sdk.dispose()
        
    def start_drag(self, event):

        """
        Starts dragging the centered position of the zoomed in rendered image
        """

        self.dragging = True
        self.drag_start_x = event.x
        self.drag_start_y = event.y

    def drag_image(self, event):

        """
        Drags the the centered position of the zoomed in rendered image and updates the frame
        """

        if not self.dragging or self.last_frame is None:
            return
        
        dx = event.x - self.drag_start_x
        dy = event.y - self.drag_start_y

        h, w = self.last_frame.shape
        label_w = self.image_label.winfo_width()
        label_h = self.image_label.winfo_height()

        if self.pixel_mode:
            self.pan_x -= int(dx / self.zoom_factor)
            self.pan_y -= int(dy / self.zoom_factor)
        else:
            scale_x = w / label_w
            scale_y = h / label_h

            self.pan_x -= int(dx * scale_x / self.zoom_factor)
            self.pan_y -= int(dy * scale_y / self.zoom_factor)

        self.drag_start_x = event.x
        self.drag_start_y = event.y

        self.render_image(self.last_frame)

    def stop_drag(self, event):

        """
        Stops dragging the centered position
        """

        self.dragging = False

    def set_exposure_from_entry(self):

        """
        Sets the exposure of the camera from user-entry
        """

        try:
            value = float(self.exposure_var.get())
            if value <= 0:
                raise ValueError

            was_live = self.live_running
            if was_live:
                self.camera.disarm()

            self.camera.exposure_time_us = int(float(value) * 1000)

            if was_live:
                self.camera.arm(2)
                self.camera.issue_software_trigger()
        except ValueError:
            messagebox.showerror("Error", "Enter a valid exposure time")
        except Exception as e:
            print("Exposure error:", e)

    def toggle_pixel_mode(self):

        """
        Toggles 1:1 pixel mode in the rendered image
        """

        self.pixel_mode = not self.pixel_mode
        
        if self.pixel_mode:
            self.pixel_button.config(text="Fit to Window")
        else: 
            self.pixel_button.config(text="1:1 Pixels")

        if self.last_frame is not None:
            self.render_image(self.last_frame)

    def toggle_autostretch(self):

        """
        Toggles autostretch from button push
        """

        self.auto_stretch = not self.auto_stretch

        if self.auto_stretch:
            self.display_min = self.current_min
            self.display_max = self.current_max
            self.updating_sliders
            self.auto_stretch_button.config(text="Autostretch: ON")
        else:
            self.auto_stretch_button.config(text="Autostretch: OFF")
        
        if self.last_frame is not None:
            self.render_image(self.last_frame)
    
    def update_from_min_max(self):
        
        """
        Updates brightness and contrast from the movement of the minimum and maximum portrayed pixel intensity values.
        """

        center = (self.display_min + self.display_max) / 2
        width = (self.display_max - self.display_min)

        brightness = (center - 32767.5) / 32767.5
        contrast = width / 65535

        self.updating_sliders = True
        self.brightness_slider.set(brightness)
        self.contrast_slider.set(contrast)
        self.updating_sliders = False

    def min_change(self, value):

        """
        Changing the minimum portrayed pixel intensity value in the rendered image
        """

        self.auto_stretch = False
        self.auto_stretch_button.config(text="Autostretch: OFF")

        if self.updating_sliders:
            return
        
        self.display_min = float(value)

        if self.display_min >= self.display_max:
            self.display_min = self.display_max - 1

        self.update_from_min_max()

    def max_change(self, value):

        """
        Changing the maximum portrayed pixel intensity value in the rendered image
        """

        self.auto_stretch = False
        self.auto_stretch_button.config(text="Autostretch: OFF")

        if self.updating_sliders:
            return
        
        self.display_max = float(value)

        if self.display_max <= self.display_min:
            self.display_max = self.display_min + 1

        self.update_from_min_max()

    def brightness_change(self, value):

        """
        Changing the brightness of the rendered image
        """

        self.auto_stretch = False
        self.auto_stretch_button.config(text="Autostretch: OFF")

        if self.updating_sliders:
            return
        
        brightness = float(value)

        width = self.display_max - self.display_min
        center = 32767.5 + brightness * 32767.5

        self.display_min = center - width / 2
        self.display_max = center + width / 2

        self.display_min = max(0, self.display_min)
        self.display_max = min(65535, self.display_max)

        self.updating_sliders = True
        self.min_slider.set(self.display_min)
        self.max_slider.set(self.display_max)
        self.updating_sliders = False

    def contrast_change(self, value):

        """
        Changing the contrast of the rendered image
        """

        self.auto_stretch = False
        self.auto_stretch_button.config(text="Autostretch: OFF")
        if self.updating_sliders:
            return
        
        contrast = float(value)

        center = (self.display_min + self.display_max) / 2
        width = 65535 * contrast

        self.display_min = center - width / 2
        self.display_max = center + width / 2

        self.display_min = max(0, self.display_min)
        self.display_max = min(65535, self.display_max)

        self.updating_sliders = True
        self.min_slider.set(self.display_min)
        self.max_slider.set(self.display_max)
        self.updating_sliders = False

    def auto_contrast(self):

        """
        Autosets the minimum and maximum portrayed pixel intensity values by moving the sliders to the optimized positions
        """

        if self.last_frame is None:
            return
        
        img = self.last_frame.astype(np.float32)

        self.display_min = np.percentile(img, 0.5)
        self.display_max = np.percentile(img, 99.8)

        self.min_slider.set(self.display_min)
        self.max_slider.set(self.display_max)

        self.update_from_min_max()

    def reset_contrast(self):

        """
        Resets the sliders for minimum and maximum portrayed pixel intensity values to the original values
        """

        if self.last_frame is None:
            return
        
        self.display_min = self.initial_display_min
        self.display_max = self.initial_display_max

        self.current_min = self.initial_display_min
        self.current_max = self.initial_display_max

        self.min_slider.set(self.display_min)
        self.max_slider.set(self.display_max)

        self.update_from_min_max()
        self.update_sliders_range()

    def set_min_max(self):
        
        """
        Sets the minimum and maximum portrayed pixel intensity values from user-entered values
        """

        set_min_max_window = tk.Toplevel(self.root)
        set_min_max_window.geometry("230x110")
        set_min_max_window.title("Set Minimum and Maximum Pixel Value")

        tk.Label(set_min_max_window, text="Minimum Pixel Value:").grid(row=0, column=0, padx=10, pady=5)
        tk.Label(set_min_max_window, text="Maximum Pixel Value:").grid(row=1, column=0, padx=10, pady=5)

        min_var = tk.StringVar(value=str(self.current_min))
        max_var = tk.StringVar(value=str(self.current_max))

        min_entry = tk.Entry(set_min_max_window, textvariable=min_var, width=10)
        min_entry.grid(row=0, column=1, padx=10, pady=5)

        max_entry = tk.Entry(set_min_max_window, textvariable=max_var, width=10)
        max_entry.grid(row=1, column=1, padx=10, pady=5)

        def set_values():
            try:
                min_value = float(min_var.get())
                max_value = float(max_var.get())

                if min_value >= max_value:
                    raise ValueError("Maximum must be larger than Minimum")
            
                if min_value < 0:
                    raise ValueError("Minimum can not be less than zero")
            
                if max_value > 65535:
                    raise ValueError("Maximum can not be larger than 65535")
            
                self.current_min = min_value
                self.current_max = max_value

                self.display_min = max(self.display_min, self.current_min)
                self.display_max = min(self.display_max, self.current_max)

                print(f"Minimum changed to {min_value} \n Maximum changed to {max_value}")

                self.update_sliders_range()

                set_min_max_window.destroy()
            except Exception as e:
                messagebox.showerror("Invalid input:", str(e))

        tk.Button(set_min_max_window, text="Set values", command=set_values).grid(row=2, column=0, pady=10)
        tk.Button(set_min_max_window, text="Cancel", command=set_min_max_window.destroy).grid(row=2, column=1, pady=10)

    def update_sliders_range(self):

        """
        Updates sliders from user-manipulation in the brightness and contrast frame
        """

        self.min_slider.config(
            from_=self.current_min,
            to=self.current_max
        )

        self.max_slider.config(
            from_=self.current_min,
            to=self.current_max
        )

        self.brightness_slider.config(
            from_=self.current_min,
            to=self.current_max
        )

        self.contrast_slider.config(
            from_=1,
            to=(self.current_max-self.current_min)
        )

        self.update_sliders()

    def update_sliders(self):

        """
        Updates values from moving the sliders in the brightness and contrast frame
        """

        self.min_slider.set(self.display_min)
        self.max_slider.set(self.display_max)

        brightness = (self.display_max + self.display_min) / 2
        contrast = (self.display_max - self.display_min)

        self.brightness_slider.set(brightness)
        self.contrast_slider.set(contrast)
    
    def capture_calibration_image(self):

        """
        Capture image in calibration mode
        """

        if self.save_directory is None:
            messagebox.showwarning("Warning",
                                   "Select save directory first.")
            return
        
        if self.last_frame is None:
            print("No frame available")

        self.pause_live()

        pixels = self.last_frame.copy()

        img = Image.fromarray(pixels)

        answer = messagebox.askyesno("Save Image?",
                                     "Do you want to save this image?")

        if answer:
            try:
                sam_pos = self.sample_position 
                cam_pos = self.camera_position
            except ValueError:
                messagebox.showerror("Error", "Invalid sample or camera position")
                return
            
            filename = f"Cal_Sam_{sam_pos:.6f}mm_Cam_{cam_pos:.6f}mm_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".tif"
            path = os.path.join(self.save_directory, filename)
            img.save(path)
            print("Saved:", path)

        # Use image for calibration
        answer2 = messagebox.askyesno("Calibration", "Do you want to use this image for calibration of camera step size?")

        if answer2:
            try:
                sam_pos = self.sample_position
                cam_pos = self.camera_position
            except ValueError:
                messagebox.showerror("Error", "Invalid sample or camera position")
                return

            self.sample_positions.append(sam_pos)
            self.camera_positions.append(cam_pos)

        self.resume_live()

    def pause_live(self):

        """
        Pause live imaging
        """

        if self.live_running:
            self.live_running = False
            self.camera.disarm()

    def resume_live(self):

        """
        Resume live imaging
        """

        self.camera.arm(2)
        self.camera.issue_software_trigger()
        self.live_running = True
        self.update_live()

    def save_image(self):

        """
        Save image
        """
        if self.save_directory is None:
            messagebox.showwarning("Warning",
                                   "Select save directory first.")
            return

        self.update_live()

        pixels = self.last_frame.copy()

        img = Image.fromarray(pixels)

        # Save image
        answer = messagebox.askyesno("Save Image?",
                                     "Do you want to save this image?")

        if answer:
            try:
                sam_pos = self.sample_position
                cam_pos = self.camera_position
            except ValueError:
                messagebox.showerror("Error", "Invalid sample or camera position")
                return
            
            filename = f"Cal_Sam_{sam_pos:.6f}mm_Cam_{cam_pos:.6f}mm_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".tif"
            path = os.path.join(self.save_directory, filename)
            img.save(path)
            print("Saved:", path)
        
        self.update_live()

    def move_sample_from_entry(self):

        """
        Moves the motorized stage of the sample to the user-entered position
        """

        try:
            value = float(self.sample_position_var.get())

            if not 0 <= value <= 23:
                messagebox.showerror("Error", "Allowed range: 0–23 mm")
                return

            self.sample_position = value
            self.move_absolute_mm(self.serial_1, value)

        except ValueError:
            messagebox.showerror("Error", "Please enter a valid number")
    
    def move_camera_from_entry(self):

        """
        Moves the motorized stage of the camera to the user-entered position
        """

        try:
            value = float(self.camera_position_var.get())

            if not 0 <= value <= 23:
                messagebox.showerror("Error", "Allowed range: 0–23 mm")
                return

            self.camera_position = value
            self.move_absolute_mm(self.serial_2, value)

        except ValueError:
            messagebox.showerror("Error", "Please enter a valid number")

    def find_best_camera_position(self):

        """
        Find best camera position using the selected image quality analyzer
        The code is currently written so that:
            1. 5 positions within a ±0.2 cm range of the current camera position are analyzed with the selected image quality
            analyzer. The best position serves as a starting point for the next loop
            2. 5 positions within a ±0.1 cm range of the current camera position are analyzed with the selected image quality
            analyzer. The best position serves as a starting point for the next loop
            3. 11 positions within a ±0.05 cm range of the current camera position are analyzed with the selected image 
            quality analyzer. A second order plot is fitted to the image quality scores and the camera positions to find the 
            best camera position. Another second order plot is fitted to the top three values. A graph is shown of the image
            quality scores and the camera positions so that the user can decide themselves which camera position they want to 
            use to take the best image.
        The motorized stage of the camera then moves to the best position (if the best position has been accurately 
        calculated).
        """
        
        self.update_live()

        pixels = self.last_frame.copy()

        img = Image.fromarray(pixels)

        starting_position = self.camera_position

        # testing 5 positions within a -0.2 to a +0.2 cm interval to find the best camera position
        t_meas_start = time.time()
        print("Starting first loop")
        offsets1 = np.array([-0.2, -0.1, 0.0, 0.1, 0.2])
        x1, y1, best1 = self.image_loop_for_best_position(starting_position, offsets1)
        
        # testing 5 positions within a -0.1 to a +0.1 cm interval to find the best camera position
        offsets2 = np.array([-0.1, -0.05, 0.0, 0.05, 0.1])
        print("Starting second loop")
        x2, y2, best2 = self.image_loop_for_best_position(best1, offsets2)

        # testing 11 positions within a -0.05 to a +0.05 cm interval to find the best camera position
        offsets3 = np.array([-0.05, -0.04, -0.03, -0.02, -0.01, 0.0, 0.01, 0.02, 0.03, 0.04, 0.05])
        print("Starting third loop")
        x3, y3, best3 = self.image_loop_for_best_position(best2, offsets3)

        t_meas_end = time.time()
        print(f"Time to produce one calibrated image: {t_meas_end - t_meas_start} seconds")

        # Plotting --------------------------------------------------------------------------------------------
        plot_window = tk.Toplevel(self.root)
        plot_window.title("Best Camera Position Plot")

        fig, ax = plt.subplots()

        # 10 Measured points ----------------------------------------------------------------------------------
        x = np.array(x3, dtype=float)
        y = np.array(y3, dtype=float)

        ax.scatter(x, y, label="Measured Points")

        # Fitting to the measured points ----------------------------------------------------------------------
        def second_order_fit(x_values, y_values):
            coeffs = np.polyfit(x_values, y_values, 2)
            a, b, c = coeffs

            x_peak_value = -b / (2*a)
            y_peak_value = np.polyval(coeffs, x_peak_value)

            return x_peak_value, y_peak_value
        
        coeffs10 = np.polyfit(x, y, 2)

        x_fit10 = np.linspace(x.min(), x.max(), 400)
        y_fit10 = np.polyval(coeffs10, x_fit10)

        ax.plot(x_fit10, y_fit10, linestyle=':', label="Fit (Measured points)")

        # Marking the highest fitted point with an "x" and a label that says: "Peak (10 values fitting curve)"

        peak10_x, peak10_y = second_order_fit(x, y)

        ax.plot(peak10_x, peak10_y, 'rx', markersize=12, label="Peak (Measured points fit)")

        # Fit to the top three highest measured points --------------------------------------------------------

        highest_value = np.argmax(y)
        
        #Top three points
        x_3 = x[highest_value-1:highest_value+2]
        y_3 = y[highest_value-1:highest_value+2]

        coeffs3 = np.polyfit(x_3, y_3, 2)

        x_fit3 = np.linspace(x_3.min(), x_3.max(), 200)
        y_fit3 = np.polyval(coeffs3, x_fit3)

        ax.plot(x_fit3, y_fit3, linestyle='--', label="Fit (Top 3 values)")

        # Marking the highest fitted point with an "x" and a label that says: "Peak (top 3 values fitting curve)"

        peak3_x, peak3_y = second_order_fit(x_3, y_3)

        ax.plot(peak3_x, peak3_y, 'kx', markersize=12, label="Peak (Top 3 values fit)")

        # The best position for the peak with 3 values (x-value) is returned
        #   If this peak is out of range (of the 3 values) then the peak of the 10 values fitting curve is used
        #       If the peak is out of range (for the 10 values) then a variable called best_position is returned
        
        best_position = best3

        def in_range(x_min, x_max, value):
            if x_min <= value <= x_max:
                return True
            return False
            
        if in_range(x_3.min(), x_3.max(), peak3_x):
            print("Calculated best position from fit of top 3 values")
            final_best_position = peak3_x
            final_best_value = peak3_y
        elif in_range(x.min(), x.max(), peak10_x):
            print("Calculated best position from fit of top 3 values")
            print("Calculated best position from fit of top 3 values out of range \nProceeding with fit of measured values")
            print("Calculated best position from fit of measured values")
            final_best_position = peak10_x
            final_best_value = peak10_y
        else: 
            print("Calculated best position from fit of top 3 values")
            print("Calculated best position from fit of top 3 values out of range \nProceeding with fit of measured values")
            print("Calculated best position from fit of measured values")
            print("Calculated best position from fit of measured values out of range \nProceeding with top measured value")
            final_best_position = best_position
            final_best_value = y[highest_value]

        peak3 = float(peak3_x)
        peak10 = float(peak10_x)
        best_pos = float(best_position)

        print(f"3 point fit             Best position: {peak3:.6f}, Peak value: {peak3_y}")
        print(f"Measured values fit     Best position: {peak10:.6f}, Peak value: {peak10_y}")
        print(f"Best measured position  Best position: {best_pos:.6f}, Peak value: {y[highest_value]}")
        print()
        print(f"Camera at position: {final_best_position}")
        print(f"Peak value: {final_best_value}")

        # -----------------------------------------------------------------------------------------------------------------------------

        ax.set_xlabel("Camera position (mm)")
        ax.set_ylabel("Focus Score")
        ax.set_title("Focus Score vs. Camera position")

        ax.grid(True)
        ax.legend()

        canvas = FigureCanvasTkAgg(fig, master=plot_window)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="both", expand=True)

        # -----------------------------------------------------------------------------------------------------------------------------

        def close_window():
            plt.close(fig)
            plot_window.destroy()
        
        plot_window.protocol("WM_DELETE_WINDOW", close_window)

        # Move to the calculated best position
        self.move_absolute_mm(self.serial_2, final_best_position)
        self.camera_position = self.get_position_mm(self.serial_2)
        self.camera_position_var.set(str(final_best_position))

        self.update_live()

    def use_for_calibration(self):
        
        """
        Asks the user if they want to save the current positions of the sample and camera stage for calibration of the 
        camera step size
        """
        answer2 = messagebox.askyesno("Calibration", "Do you want to use this image for calibration of camera step size?")

        if answer2:
            try:
                sam_pos = self.get_position_mm(self.serial_1)
                cam_pos = self.get_position_mm(self.serial_2)
            except ValueError:
                messagebox.showerror("Error", "Invalid sample or camera position")
                return

            self.sample_positions.append(sam_pos)

            self.camera_positions.append(cam_pos)

            print(f"Sample at: {sam_pos} mm and Camera at {cam_pos} mm has been added for calibration")

    def capture_image_with_iqa(self):

        """
        Capture image with image image quality analyzer
        """

        self.find_best_camera_position()
        self.use_for_calibration()
        self.save_image()

    def image_loop_for_best_position(self, center_position, offsets):

        """
        Imaging loop to find the best camera position
        Used in find_best_camera_position()
        """
        positions = []
        values = []

        best_value = 0
        best_position = center_position

        for offset in offsets:
            position = center_position + offset
            self.move_absolute_mm(self.serial_2, position)

            time.sleep(0.25)

            self.update_live()
            time.sleep(0.1)
            self.update_live()

            pixels = self.last_frame.copy()

            value, _ = self.image_quality_analysis(pixels)

            print(f"Position: {position}, Value: {value}")

            positions.append(position)
            values.append(value)

            if value > best_value:
                best_value = value
                best_position = position

        return np.array(positions), np.array(values), best_position

    def image_quality_analysis(self, pixels):

        """
        Image quality analyzing methods used for finding the best camera position
        """
            
        value = 0
        img = 0
        image_analysis_var = self.mode_var.get()

        # Alternatives for image quality testing:
            # 1. Brenner gradient (simple, fast, noise resistant)
            # 2. Tenengrad (high sensitivity, noise resistant, efficient)
            # 3. Laplacian Variance (simple, fast, effective)
            # 4. FFT High-Frequency Energy (extremely accurate, slower)

        # Brenner gradient image quality testing --------------------------------------------
        if image_analysis_var == "Brenner gradient":
            h, w = pixels.shape
            img = pixels[h//4:3*h//4, w//4:3*w//4]

            img = img.astype(np.float32)

            diff = img[:, 2:] - img[:, :-2]
            value = np.mean(diff**2)
            return value, img
        # -----------------------------------------------------------------------------------

        # Tenengrad image quality testing ---------------------------------------------------
        if image_analysis_var == "Tenengrad":
            h, w = pixels.shape
            img = pixels[h//4:3*h//4, w//4:3*w//4]

            img = img.astype(np.float32)

            gx = cv2.Sobel(img, cv2.CV_32F, 1, 0, ksize=3)
            gy = cv2.Sobel(img, cv2.CV_32F, 0, 1, ksize=3)

            g2 = gx**2 + gy**2
            value = np.mean(g2)
            return value, img
        #------------------------------------------------------------------------------------

        # Laplacian variance image quality testing ------------------------------------------
        if image_analysis_var == "Laplacian variance":
            h, w = pixels.shape
            pixels = pixels[h//4: 3*h//4, w//4:3*w//4]

            img = cv2.GaussianBlur(pixels, (3,3), 0)
            lap = cv2.Laplacian(img, cv2.CV_64F)

            value = lap.var()
            return value, img
        # -----------------------------------------------------------------------------------

        # FFT High-Frequence Energy ---------------------------------------------------------
        if image_analysis_var == "Fast FT":
            h, w = pixels.shape
            img = pixels[h//4:3*h//4, w//4:3*w//4]

            img = img.astype(np.float32)

            f = np.fft.fft2(img)
            fshift = np.fft.fftshift(f)

            magnitude = np.abs(fshift)

            h, w = magnitude.shape

            # remove low frequencies
            magnitude[h//2-20:h//2+20, w//2-20:w//2+20] = 0

            value = np.mean(magnitude)
            return value, img
        # -----------------------------------------------------------------------------------

        print("No image quality analysis tool was used")
        return value, img

    def show_calibration_plots(self):

        """
        Shows the calibration plot in a new window
        The window shows the relationship between the sample and the camera stage
        A first order fit is fitted to the entered positions
        The calibration coefficient of the camera step size is shown as the gradient of the first order fit
        """
        if len(self.sample_positions) <= 1:
            messagebox.showwarning("Warning", "More data is required for plotting.")
            return
        
        plot_window = tk.Toplevel(self.root)
        plot_window.title("Calibration Plot")

        fig, ax = plt.subplots()

        x = [float(v) for v in self.sample_positions]
        y = [float(v) for v in self.camera_positions]

        ax.scatter(x, y, label="Measured Points")

        # First Order Fit
        coeffs = np.polyfit(x, y, 1)
        a = coeffs[0]
        b = coeffs[1]

        x_lin_fit = np.linspace(min(x), max(x), 100)
        y_lin_fit = a * x_lin_fit + b

        ax.plot(x_lin_fit, y_lin_fit, linestyle='--', label="Linear Fit")

        equation_text = f"y = {a:.6f}x + {b:.6f}"
        ax.text(0.05, 0.95, equation_text, transform=ax.transAxes, verticalalignment='top')

        ax.set_xlabel("Sample position (mm)")
        ax.set_ylabel("Camera position (mm)")
        ax.set_title("Sample vs. Camera position")

        ax.grid(True)
        ax.legend()

        canvas = FigureCanvasTkAgg(fig, master=plot_window)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="both", expand=True)

        def close_window():
            plt.close(fig)
            plot_window.destroy()
        
        plot_window.protocol("WM_DELETE_WINDOW", close_window)

    def toggle_beam_blocker(self):

        """
        Toggles the beam blocker by writing commands to the Arduino hardware
        """

        beam_blocker_var = not self.beam_blocker

        if beam_blocker_var:
            answer = messagebox.askyesno("Warning",
                                     "You are about to remove the beam blocker. Put on safety glasses and make sure that no one can get hurt by the laser. Do you want to proceed?")

            if answer:
                try:
                    self.arduino.write(b'U')
                except:
                    messagebox.showerror("Error", "The beam blocker could not be removed.")
                    return
                
                self.beam_blocker_button.config(text="Beam Blocker Status: NOT BLOCKING")
                self.beam_blocker = beam_blocker_var
        else:
            try:
                self.arduino.write(b'D')
            except:
                messagebox.showerror("Error", "Beam blocker could not be reset to BLOCKING.")
                return
            self.beam_blocker_button.config(text="Beam Blocker Status: BLOCKING")
            self.beam_blocker = beam_blocker_var
    
    def select_laser(self, wavelength):

        """
        Selects active wavelengths by listening to button presses from the user
        """

        state = self.laser_vars[wavelength].get()

        if state == 1:
            self.skyra.select_wavelength(wavelength)
        else:
            self.skyra.deselect_wavelength(wavelength)
    
    def set_laser_power(self):

        """
        Sets laser power of the selected wavelengths from user entry
        """
        power = float(self.power_var.get())
        self.skyra.set_power(power)

        print(f"Power set to {power} mW")

    def toggle_laser(self):

        """
        Toggles laser emission from user entry
        """
        laser_var_temp = not self.laser_var

        if laser_var_temp:
            answer = messagebox.askyesno("Warning",
                                     "You are about to turn on the laser. Put on safety glasses and make sure that no one can get hurt by the laser. Do you want to proceed?")

            if answer:
                try:
                    self.skyra.emission_on()
                    print("Turning selected laser(s) ON")
                except:
                    messagebox.showerror("Error", "The laser could not be turned on.")
                    return

                self.toggle_laser_button.config(text="Laser Status: ON")
                self.laser_var = laser_var_temp
        else:
            try:
                self.skyra.emission_off()
                print("Turning selected laser(s) OFF")
            except:
                messagebox.showerror("Error", "The laser(s) could not be turned OFF.")
                return
            
            self.toggle_laser_button.config(text="Laser Status: OFF")
            self.laser_var = laser_var_temp

    def select_directory(self):

        """
        Selects saving directory of images based on user entry
        """

        self.save_directory = filedialog.askdirectory()

    def exit_calibration(self):

        """
        Exit calibration mode
        Takes the user back to the main menu
        Camera is disarmed, beam blocker is set to blocking the laser, and the laser emission is turned off
        """

        print("Exiting calibration")
        
        # Live imaging is turned off
        self.live_running = False

        if self.live_job is not None:
            self.root.after_cancel(self.live_job)
            self.live_job = None

        # Camera dismantled
        self.camera.disarm()
        
        # Beam blocker set to blocking
        self.arduino.write(b'D')
        self.beam_blocker = True

        # Laser emission off
        self.skyra.emission_off()

        # Back to main menu
        self.build_menu()

    # -----------------------------------------------------------------------------------------------------------------------------

    # -----------------------------------------------------------------------------------------------------------------------------
    # Measurement
    # -----------------------------------------------------------------------------------------------------------------------------

    def enter_measurement(self):

        """
        Enter measurement mode
        Builds the window where z-stacks can be acquired through either normal z-stack image acquisition or z-stack 
        acquisition with image quality analysis
        """

        self.clear_container()

        # Window settings
        tk.Label(self.container, text="Measurement Mode",
                 font=("Arial", 14)).grid(row=0, column=0, columnspan=2, pady=10)

        self.measurement_frame = tk.Frame(self.container)
        self.measurement_frame.grid(row=1, column=0, columnspan=2, sticky="nsew")

        self.measurement_frame.grid_rowconfigure(1, weight=1)
        self.measurement_frame.grid_columnconfigure(0, weight=1, uniform="half")
        self.measurement_frame.grid_columnconfigure(1, weight=1, uniform="half")

        # -------------------------------------------------------------------------------------------------------------------------------
        
        # -------------------------------------------------------------------------------------------------------------------------------
        # Left frame
        # Regular z-stack acquisition

        self.left_frame = tk.Frame(self.measurement_frame)
        self.left_frame.grid(row=1, column=0, sticky="nsew", padx=10)
        self.left_frame.grid_propagate(False)

        self.normal_measurement = tk.LabelFrame(self.left_frame, text="Regular Measurement", padx=5, pady=5)
        self.normal_measurement.pack(fill="both", expand=True, pady=(5, 10))

        # User entries

        # Starting position sample stage
        tk.Label(self.normal_measurement,
                    text="Start Position - Sample").pack()
        
        self.start_position_sample_var = tk.StringVar(value=str(self.start_position_sample))

        self.start_position_sample_entry = tk.Entry(self.normal_measurement, textvariable=self.start_position_sample_var)
        self.start_position_sample_entry.pack()
        
        # Starting position camera stage
        tk.Label(self.normal_measurement,
                    text="Start Position - Camera").pack()
        
        self.start_position_camera_var = tk.StringVar(value=str(self.start_position_camera))

        self.start_position_camera_entry = tk.Entry(self.normal_measurement, textvariable=self.start_position_camera_var)
        self.start_position_camera_entry.pack()
        
        # Step size of the sample stage
        tk.Label(self.normal_measurement,
                    text="Step Size for Imaging (mm)").pack()
        
        tk.Entry(self.normal_measurement,
                     textvariable=self.step_size).pack()

        # Number of images
        tk.Label(self.normal_measurement,
                    text="Number of Images").pack()
        
        tk.Entry(self.normal_measurement,
                     textvariable=self.number_of_images).pack()
        
        # Calibration coefficient of the camera step size
        tk.Label(self.normal_measurement, 
                     text="Calibration coefficient for the camera step size").pack()
        
        self.cal_coeff_var = tk.StringVar(value=str(self.cal_coeff))
        self.cal_coeff_entry = tk.Entry(self.normal_measurement,
                     textvariable=self.cal_coeff_var)
        self.cal_coeff_entry.pack()
        
        # Laser settings
        self.laser_options_frame = tk.LabelFrame(self.normal_measurement, text="Laser Options")
        self.laser_options_frame.pack(padx=10, pady=10)

        for i, wl in enumerate([405, 488, 561, 638]):

            tk.Checkbutton(
                self.laser_options_frame,
                text=f"{wl} nm",
                variable=self.laser_vars[wl],
                command=lambda w=wl: self.select_laser(w)
            ).grid(row=1, column=i, padx=5, pady=(5,5))

        for i in range(len(self.laser_vars)):
            self.laser_options_frame.grid_columnconfigure(i, weight=1)

        tk.Label(self.laser_options_frame, text="Power (mW)").grid(row=2, column=0, padx=5, pady=(5,5))

        tk.Entry(self.laser_options_frame, textvariable=self.power_var, width=50).grid(row=2, column=1, columnspan=3, padx=5, pady=(5,5))

        # Start regular measurement
        tk.Button(self.normal_measurement, 
                  text="Start Measurement", 
                  command=self.start_measurement).pack(pady=3)
        
        # -------------------------------------------------------------------------------------------------------------------------------

        # -------------------------------------------------------------------------------------------------------------------------------
        # Right frame
        # Z-stack acquisiton with image quality analysis

        self.right_frame = tk.Frame(self.measurement_frame)
        self.right_frame.grid(row=1, column=1, sticky="nsew", padx=10)
        self.right_frame.grid_propagate(False)

        self.iqa_measurement = tk.LabelFrame(self.right_frame, text="Measurement with Image Quality Analysis", padx=5, pady=5)
        self.iqa_measurement.pack(fill="both", expand=True, pady=(5,10))

        # User entries

        # Starting position sample stage
        tk.Label(self.iqa_measurement,
                    text="Start Position - Sample").pack()
        
        self.start_position_sample_var = tk.StringVar(value=str(self.start_position_sample))
        self.start_position_sample_entry = tk.Entry(self.iqa_measurement, textvariable=self.start_position_sample_var)
        self.start_position_sample_entry.pack()

        # Starting position camera stage
        tk.Label(self.iqa_measurement,
                    text="Start Position - Camera").pack()
        
        self.start_position_camera_var = tk.StringVar(value=str(self.start_position_camera))
        self.start_position_camera_entry = tk.Entry(self.iqa_measurement, textvariable=self.start_position_camera_var)
        self.start_position_camera_entry.pack()

        # Step size of the sample stage
        tk.Label(self.iqa_measurement,
                    text="Step Size for Imaging (mm)").pack()
        
        tk.Entry(self.iqa_measurement,
                     textvariable=self.step_size).pack()

        # Number of images
        tk.Label(self.iqa_measurement,
                    text="Number of Images").pack()
        
        tk.Entry(self.iqa_measurement,
                     textvariable=self.number_of_images).pack()
        
        # Calibration coefficient of the camera step size
        tk.Label(self.iqa_measurement, 
                     text="Calibration coefficient for the camera step size").pack()
        
        self.cal_coeff_var = tk.StringVar(value=str(self.cal_coeff))
        self.cal_coeff_entry = tk.Entry(self.iqa_measurement,
                     textvariable=self.cal_coeff_var)
        self.cal_coeff_entry.pack()
    
        # Laser settings
        self.laser_options_frame2 = tk.LabelFrame(self.iqa_measurement, text="Laser Options")
        self.laser_options_frame2.pack(padx=10, pady=10)

        for i, wl in enumerate([405, 488, 561, 638]):

            tk.Checkbutton(
                self.laser_options_frame2,
                text=f"{wl} nm",
                variable=self.laser_vars[wl],
                command=lambda w=wl: self.select_laser(w)
            ).grid(row=1, column=i, padx=5, pady=(5,5))

        for i in range(len(self.laser_vars)):
            self.laser_options_frame2.grid_columnconfigure(i, weight=1)

        tk.Label(self.laser_options_frame2, text="Power (mW)").grid(row=2, column=0, padx=5, pady=(5,5))

        tk.Entry(self.laser_options_frame2, textvariable=self.power_var, width=50).grid(row=2, column=1, columnspan=3, padx=5, pady=(5,5))

        # Image quality analyzers
        image_analysis_options = ["Brenner gradient", "Fast FT", "Laplacian variance", "Tenengrad"]
        image_analysis_frame = tk.LabelFrame(self.iqa_measurement, text="Image Quality Analyzers")
        image_analysis_frame.pack(padx=10, pady=10)

        for i, opt in enumerate(image_analysis_options):
            tk.Radiobutton(
                image_analysis_frame,
                text=opt,
                variable=self.mode_var,
                value=opt
            ).grid(row=0, column=i, padx=5)

        for i in range(len(image_analysis_options)):
            image_analysis_frame.grid_columnconfigure(i, weight=1)
        
        # Start measurement with image quality analysis
        tk.Button(self.iqa_measurement, 
                  text="Start Measurement with Image Quality Analysis",
                  command=self.start_iqa_measurement).pack(pady=3)

        # Threshold for the fast version of measurement with image quality analysis
        tk.Label(self.iqa_measurement,
                    text="Threshold (%) for Measurement with Image Quality Analysis - Fast Version").pack(pady=(9,3))
        
        self.threshold_var = tk.StringVar(value=str(self.threshold_percentage))
        self.threshold_entry = tk.Entry(self.iqa_measurement, textvariable=self.threshold_var)
        self.threshold_entry.pack(pady=3) 

        # Start fast version of measurement with image quality analysis
        tk.Button(self.iqa_measurement,
                  text="Start Measurement with Image Quality Analysis - Fast Version",
                  command=self.start_fast_measurement).pack(pady=3)
        
        # -------------------------------------------------------------------------------------------------------------------------
        
        # Buttons in the bottom of the window
        self.other_frame = tk.Frame(self.measurement_frame)
        self.other_frame.grid(row=2, column=0, columnspan=2, padx=10)

        # Info
        tk.Button(self.other_frame, text="Info",
                  command=self.info).grid(row=0, column=0, padx=10, pady=(10,5))

        # Select directory for saving images
        tk.Button(self.other_frame, text="Select Save Directory",
                  command=self.select_directory).grid(row=1, column=0, padx=10, pady=(5,5)) #pack(pady=5)

        # Back to main menu
        tk.Button(self.other_frame, text="Back to Menu",
                  command=self.build_menu).grid(row=2, column=0, padx=10, pady=(5,10)) #.pack(pady=10)
    
    def start_measurement(self):

        """
        Starts a regular z-stack acquisition
        """
        
        # First control of valid entries
        try:
            start_sample = float(self.start_position_sample_var.get())
            start_camera = float(self.start_position_camera_var.get())
            step_size_sample = float(self.step_size.get())
            number_of_images = int(float(self.number_of_images.get()))
            cali_coeff = float(self.cal_coeff_var.get())
            self.set_laser_power()
        
        except ValueError:
            messagebox.showerror("Error", "Please enter valid numeric values")

        # Second control to validate that the z-stack is possible
        if not (0 <= start_sample <= 23):
            messagebox.showerror("Error", "Sample start position must be 0–23 mm")
            return

        if not (0 <= start_camera <= 23):
            messagebox.showerror("Error", "Camera start position must be 0–23 mm")
            return

        if step_size_sample <= 0:
            messagebox.showerror("Error", "Step size must be larger than zero")
            return

        if number_of_images <= 0:
            messagebox.showerror("Error", "Number of images must be greater than zero")
            return
        
        if cali_coeff <= 0:
            messagebox.showerror("Error", "Calibration coefficient for the camera step size must be greater than zero")
            return
        
        # Check that the image acquisition will not extend the limits of the motorized stages
        final_position_sample = start_sample + step_size_sample * number_of_images
        final_position_camera = start_camera + cali_coeff * step_size_sample * number_of_images

        if final_position_sample > 23:
            messagebox.showerror("Error", f"Sample will exceed 23 mm (final = {final_position_sample:.3f}) mm")
            return
        
        if final_position_camera > 23:
            messagebox.showerror("Error", f"Camera will exceed 23 mm (final = {final_position_camera:.3f}) mm")
            return
        
        if self.save_directory is None:
            messagebox.showwarning("Warning",
                                   "Select save directory first.")
            return

        # Inform the user that the laser emission is about to be turned on
        answer_meas = messagebox.askyesno("Warning", "You are about to start a measurement. Put on safety glasses and make sure that no one can get hurt by the laser. Do you want to proceed?")
        if not answer_meas:
            return

        self.cal_coeff = cali_coeff

        # Z-stack acquisition can start
        print("Starting measurement...")

        # Laser emission on
        self.skyra.emission_on()
        
        # Beam blocker removed
        self.arduino.write(b'U')

        # Camera initialization
        self.camera_window = tk.Toplevel(self.root)
        self.camera_window.withdraw()
        self.image_label = tk.Label(self.camera_window)

        self.start_live()

        # Move to start positions
        self.move_absolute_mm(self.serial_1, start_sample)
        self.move_absolute_mm(self.serial_2, start_camera)
        time.sleep(0.15)

        # Image-Movement Loop
        current_position_sample = start_sample
        current_position_camera = start_camera

        t1 = time.time()
        
        # If z-stack acquisition with image quality, the best starting position of the camera is calculated and image 
        # acquisition starts from there
        if self.iqa_measurement_var:
            current_position_camera = self.iqa_measurement_camera_movement(current_position_camera, cali_coeff, step_size_sample)
            print(f"current_position_camera is {current_position_camera}")
            self.move_absolute_mm(self.serial_2, current_position_camera)
            print("movement has been performed")

        for i in range(number_of_images):
            print()
            print(f"Capturing image {i}/{number_of_images}")

            # Capture image
            self.capture_measurement_image(i) 

            # Get next position to move to
            current_position_sample += step_size_sample
            current_position_camera += cali_coeff * step_size_sample

            # If z-stack acquisition with image quality analysis, the camera position is set to the calculated best position
            if self.iqa_measurement_var:
                print()
                calculated_position_camera = self.iqa_measurement_camera_movement(current_position_camera, cali_coeff, step_size_sample)
                if calculated_position_camera <= 23:
                    current_position_camera = calculated_position_camera
                else:
                    print("ERROR: MOVEMENT EXCEEDED LIMIT")
                    return

            # Move sample and camera stage
            self.move_absolute_mm(self.serial_1, current_position_sample)
            self.move_absolute_mm(self.serial_2, current_position_camera)
        
        # One last image
        self.capture_measurement_image(number_of_images)
        print()
        print(f"Capturing image {number_of_images}/{number_of_images}")
        t2 = time.time()
        measurement_time = t2-t1

        print("Measurement complete")
        print(f"Measurement took {measurement_time} seconds")

        # Beam blocker set to blocking
        self.arduino.write(b'D')

        # Laser emission is turned off
        self.skyra.emission_off()

        # Camera is turned off
        self.live_running = False

        if self.live_job is not None:
            self.root.after_cancel(self.live_job)
            self.live_job = None

        self.camera.disarm()

        # Position variables of the motorized stages are updated
        self.sample_position = self.get_position_mm(self.serial_1)
        self.camera_position = self.get_position_mm(self.serial_2)

    def start_iqa_measurement(self):

        """
        Starts a z-stack acquisition with image quality analysis
        """

        # Starts a z-stack acquisition but with a variable changed to true which enables z-stack acquisition with image quality analysis
        self.iqa_measurement_var = True
        self.start_measurement()
        self.iqa_measurement_var = False

        # Results of the z-stack acquisition in the form of numbers
        number_of_images = int(float(self.number_of_images.get()))

        print()
        if not self.iqa_fast_measurement_var:
            print("Correction performed in Measurement with Image Quality Analysis")
            print(f"Correction in    {self.counter_within_range} / {number_of_images + 1} images")
            print(f"No correction in {self.counter_out_of_range} / {number_of_images + 1} images")

            # Reset variables for new measurement
            self.counter_within_range = 0
            self.counter_out_of_range = 0

    def iqa_measurement_camera_movement(self, position, cal_coeff, step_size_var):

        """
        Handles the movement of the camera during a z-stack acquisition with image quality analysis
        """

        # If the fast version of the z-stack acquisition with image quality analysis has been selected, the program checks
        # if the image quality score is greater than the user-entered threshold
        if self.iqa_fast_measurement_var:

            # Capture image
            self.update_live()
            time.sleep(0.1)
            self.update_live()
            pixels1 = self.last_frame.copy()

            # Get score
            value1, img = self.image_quality_analysis(pixels1)

            # If it is the first image of the image stack, the image quality score of the first image will be the reference 
            # point for the threshold
            if self.image_quality_var == 0.0:
                self.image_quality_var = value1

            threshold_value = float(self.threshold_var.get()) * self.image_quality_var

            # If the score is greater than the threshold value, no calculation for the best camera position is needed
            # If the score is lower than the threshold value, calculation will be performed to find the best camera position
            print(f"Focus score before measurement = {self.image_quality_var}")
            print(f"Threshold for calibration = {threshold_value}")
            if value1 > threshold_value:
                print(f"Focus score = {value1} ... NO EXTRA CALIBRATION NEEDED!")
                self.counter_no_calibration += 1
                self.image_quality_var = value1
                return position
            else:
                print(f"Focus score = {value1} ... PROCEEDING WITH 3 POINT CALIBRATION")
                self.counter_calibration += 1

        # Calculation of the best camera position

        # Variables
        offsets = [-0.8, 0.0, 0.8]
        best_positions = []
        best_values = []
        best_position = float(position)
        cal_coeff = float(cal_coeff)
        step_size_var = float(step_size_var)
        
        # Movement-Image loop to find the best position
        for i in offsets:
            # Movement
            meas_pos = position + i * cal_coeff * step_size_var
            print(f"measured position is: {meas_pos}")

            self.move_absolute_mm(self.serial_2, meas_pos)
            
            time.sleep(0.25)

            # Capture image
            self.update_live()
            time.sleep(0.1)
            self.update_live()
            pixels = self.last_frame.copy()

            # Get image quality score
            value, img = self.image_quality_analysis(pixels)

            # Add values and positions
            best_positions.append(meas_pos)
            best_values.append(value)
        
        # Calculation of the best camera position
        x = np.array(best_positions, dtype=float)
        y = np.array(best_values, dtype=float)

        coeffs = np.polyfit(x, y, 2)
        a, b, c = coeffs

        best_position = -b / (2*a)
        best_value = np.polyval(coeffs, best_position)
        step_size_value = np.polyval(coeffs, position)

        print(f"best position is: {best_position}")

        # If the calculated camera position is within the range of tested positions, this calculated position is returned
        # If the calculated camera position is not within the range of tested positions, the original step is returned
        if best_position < position + offsets[0] * cal_coeff * step_size_var or best_position > position + offsets[-1] * cal_coeff * step_size_var:
            print("CALCULATED POSITION OUT OF RANGE! Fixing...")
            self.counter_out_of_range += 1
            self.image_quality_var = step_size_value
            return position
        else:
            self.counter_within_range += 1
            self.image_quality_var = best_value
            return best_position
    
    def start_fast_measurement(self):

        """
        Starts a fast version of the z-stack acquisition with image quality analysis
        """

        try:
            threshold = float(self.threshold_var.get())
        except ValueError:
            messagebox.showerror("Error", "Please enter valid numeric values")

        if not (0 < threshold < 1.0):
            messagebox.showerror("Error", "Threshold must be 0 < threshold < 1")
            return

        # Starts a z-stack acquisition with image quality analysis, but with a variable set to true which enables the fast
        # version of the z-stack acquisition with image quality analysis
        self.iqa_fast_measurement_var = True
        self.start_iqa_measurement()
        self.iqa_fast_measurement_var = False

        # Results of the fast version of the z-stack acquisition with image quality analysis in the form of number
        number_of_images = int(float(self.number_of_images.get()))

        print()
        print("Calibration performed in Measurement with Image Quality Analysis - Fast Version")
        print(f"Calibration in    {self.counter_calibration} / {number_of_images + 1} images")
        print(f"No calibration in {self.counter_no_calibration} / {number_of_images + 1} images")

        print()
        print("Stats for images that were calibrated")
        print(f"{self.counter_within_range} / {self.counter_calibration} images were calibrated with 3 point estimation")
        print(f"{self.counter_out_of_range} / {self.counter_calibration} images were not calibrated (only step movement was performed)")

        # Reset variables for new measurement
        self.counter_within_range = 0
        self.counter_out_of_range = 0

        self.counter_calibration = 0
        self.counter_no_calibration = 0
    
    def capture_measurement_image(self, index):
        
        """
        Captures image for z-stack acquisition
        """

        # Captures an image
        self.update_live()
        time.sleep(0.1)
        self.update_live()
        pixels = self.last_frame.copy()

        img = Image.fromarray(pixels)

        # Get positions of the sample and the camera stage
        sam_pos = self.get_position_mm(self.serial_1)
        cam_pos = self.get_position_mm(self.serial_2)

        # Save image
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        filename = f"Meas_{index}_Sam_{sam_pos:.6f}mm_Cam_{cam_pos:.6f}mm_{timestamp}.tif"
        path = os.path.join(self.save_directory, filename)

        img.save(path)
    
    def info(self):

        """
        Info about the different types of z-stack acquisitions: regular z-stack acquisition, z-stack acquisition
        with image quality analysis, and the fast version of z-stack acquisition with image quality analysis
        """

        messagebox.showinfo("Information",
                            "Measurement means imaging a z-stack of the sample on the platform where the user can "
                            "enter the starting positions of the sample and the camera (mm), the step size which is "
                            "the axial distance between each image in the sample (mm), the number of images to be "
                            "taken, and the calibration coefficient which is the calibrated step size for the "
                            "camera in regards to the imaging step size. The user can also select which wavelengths " 
                            "and their intensities they want to image the sample with. \n"
                            "\n"
                            "Normal measurement is a normal z-stack imaging process where an image is taken and the " 
                            "sample and camera moves the given distances (or camera moves the calibrated distance).\n " 
                            "\n"
                            "Measurement with Image Quality Analysis is a z-stack imaging technique where each image "
                            "is calibrated using one of the chosen image analysis alternatives (Brenner gradient, " 
                            "Fast FT, Laplacian variance, or Tenengrad) to find the best image for each step. One also " 
                            "have the option to perform the fast version of the measurement with image quality analysis " 
                            "which is a normal z-stack imaging process but calibration is only performed if the image " 
                            "analysis score falls below a certain threshold. The calibration is performed by imaging " 
                            "three points close to the current position of the camera and calculating the best position " 
                            "depending the image analysis score." 
        )

    # -----------------------------------------------------------------------------------------------------------------------------
    
    # Utilities  

    def safe_shutdown(self):

        """
        Safe shutdown of the program
        """

        print("Shutting down safely")

        # Camera disarming
        try:
            self.camera.disarm()
            self.camera.dispose()
            self.sdk.dispose()
        except:
            pass

        # Beam blocker set to blocking the laser beam
        self.arduino.write(b'D')

        # Motorized stages disconnection
        try:
            self.lib.CC_StopImmediate(self.serial_1)
            self.lib.CC_StopImmediate(self.serial_2)

            self.lib.CC_StopPolling(self.serial_1)
            self.lib.CC_StopPolling(self.serial_2)
            
            self.lib.CC_Close(self.serial_1)
            self.lib.CC_Close(self.serial_2)
        except:
            pass

        # Laser shutdown
        try:
            self.skyra.shutdown()
        except:
            pass

        # Exit window
        self.root.destroy()

    def clear_container(self):

        """
        Remove all remnants of widgets in a window
        """

        for widget in self.container.winfo_children():
            widget.destroy()

    # -----------------------------------------------------------------------------------------------------------------------------

# -----------------------------------------------------------------------------------------------------------------------------
# Run code
# -----------------------------------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("700x700")
    app = MicroscopeGUI(root)
    root.mainloop()
