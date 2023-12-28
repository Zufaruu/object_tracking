import serial
import cv2
import numpy as np

# Constant
colour = (0,255,0)              # green color
lineWidth = -1                  # -1 will result in filled circle
radius = 3                      # circle radius
point = (0,0)                   # point coordinate from mouse click
points = []                     # list of point coordinates from mouse click
num_click = 0                   # number of mouse click events
is_define_object = True         # define object mode
is_tracking = False             # tracking mode
is_killing = False              # killing mode
failure_threshold = 100         # failure threshold of frame differences
integral_x = 0                  # integral x value for PID
pre_error_x = 0                 # previous error x value for PID
integral_y = 0                  # integral y value for PID  
pre_error_y = 0                 # previous error x value for PID
min = -0.123 * 60               # min angular speed of camera (degree/s)
max = 0.123 * 60                # max angular speed of camera (degree/s)
reference_frame = 0             # reference frame for tracking
reference_bbox = (0, 0, 20, 20) # reference frame for tracking
base_vel = 0.1220740379         # smallest angular speed of camera
stop_command_str = "FF 01 0F 10 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00"        # serial command to stop the camera
init_pos_command_str = "FF 01 0F 10 02 02 02 00 00 00 00 00 00 00 00 00 00 00 00 06"    # serial command for the camera to return to initial position

# function to initialize tracker
def tracker_type(type):
    tracker_type = type

    if tracker_type == 'BOOSTING':
        tracker = cv2.legacy.TrackerBoosting_create()
    if tracker_type == 'MIL':
        tracker = cv2.TrackerMIL_create() 
    if tracker_type == 'KCF':
        tracker = cv2.TrackerKCF_create() 
    if tracker_type == 'TLD':
        tracker = cv2.legacy.TrackerTLD_create() 
    if tracker_type == 'MEDIANFLOW':
        tracker = cv2.legacy.TrackerMedianFlow_create() 
    if tracker_type == 'MOSSE':
        tracker = cv2.legacy.TrackerMOSSE_create()
    if tracker_type == "CSRT":
        tracker = cv2.TrackerCSRT_create()
    
    return tracker
 
# function for mouse events
def click(event, x,y, flags, param):
    global point, num_click, points, is_define_object, is_tracking

    # if there is left mouse click and number of that mouse click < 2, the clicked coordinates will be saved in points list
    # if the number of that mouse click == 2, the mode switch from define object to tracking
    if event == cv2.EVENT_LBUTTONDOWN and num_click < 2:
        if num_click < 2:
            num_click = num_click + 1
            point = (x,y)
            points.append(point)
            is_define_object = True
            is_tracking = False

            print(f'num_click = {num_click}')
            print(f"coordinate = ({x}, {y})")
            
        if num_click == 2:
            is_define_object = False
            is_tracking = True

    # if there is right mouse click, the mode switch from tracking to define object
    if event == cv2.EVENT_RBUTTONDOWN:
        num_click = 0
        points = []
        is_define_object = True
        is_tracking = False

# function to calculate yaw angular speed using PID
def calc_vel_PID_yaw(x_max, x_min, x_setpoint, x_pv, kp, ki, kd, dt):
    global integral_x, pre_error_x
    error = x_setpoint - x_pv

    Pout = kp * error

    integral_x = integral_x + (error * dt)
    Iout = ki * integral_x

    derivative_x = (error - pre_error_x) / dt
    Dout = kd * derivative_x
    pre_error_x = error

    vel_x = (Pout + Iout + Dout) 
    if vel_x > x_max:
        vel_x = x_max
    elif vel_x < x_min:
        vel_x = x_min
    
    return vel_x

# function to calculate pitch angular speed using PID
def calc_vel_PID_pitch(y_max, y_min, y_setpoint, y_pv, kp, ki, kd, dt):
    global integral_y, pre_error_y
    error = y_setpoint - y_pv

    Pout = kp * error

    integral_y = integral_y + (error * dt)
    Iout = ki * integral_y

    derivative_y = (error - pre_error_y) / dt
    Dout = kd * derivative_y
    pre_error_y = error

    vel_y = (Pout + Iout + Dout) 

    if vel_y > y_max:
        vel_y = y_max
    elif vel_y < y_min:
        vel_y = y_min
    
    return vel_y

# function to convert velocity values to hexadecimal values
def vel2hex(yaw, pitch):
    int_yaw = int(yaw/base_vel)
    int_pitch = int(pitch/base_vel)

    if int_yaw < 0:
        int_yaw = 65535 + int_yaw
    if int_pitch < 0:
        int_pitch = 65535 + int_pitch

    #print(f'yaw = {int_yaw}, pitch = {int_pitch}')

    hex_yaw = "{:04x}".format(int_yaw)
    hex_pitch = "{:04x}".format(int_pitch)

    Psl = hex_pitch[2:]
    Psh = hex_pitch[:2]
    Ysl = hex_yaw[2:]
    Ysh = hex_yaw[:2]

    return Psl, Psh, Ysl, Ysh

# function to convert bgr frame to grayscale
def to_grayscale(arr):
    "If arr is a color image (3D array), convert it to grayscale (2D array)."
    if len(arr.shape) == 3:
        return np.mean(arr, -1)  
    else:
        return arr

# function to normalize each pixel value 
def normalize(arr):
    rng = arr.max()-arr.min()
    amin = arr.min()
    return (arr-amin)*255/rng

# function to calculate pixel difference of reference frames and target frames for failsafe
def frame_diff(frame_bgr_ref, frame_bgr_target):
    frame_bgr_ref = to_grayscale(frame_bgr_ref.astype(float))
    frame_bgr_target = to_grayscale(frame_bgr_target.astype(float))

    diff = frame_bgr_target - frame_bgr_ref  # elementwise for scipy arrays
    m_norm = np.sum(abs(diff))  # Manhattan norm
    return m_norm * 1.0 / frame_bgr_ref.size

# function to create serial command from hexadecimal velocity input of yaw and pitch
def control_parser(Psl, Psh, Ysl, Ysh):
    '''direction parsing
       0 - 32767 bawah kiri
       65535 - 32768 atas kanan
    '''

    Header = "FF 01 0F 10"
    RM = " 00" # Roll mode | 00 No control, 01 Mode Speed, 02 Mode Speed Angle
    PM = " 01" # Pitch Mode | 00 No control, 01 Mode Speed, 02 Mode Speed Angle
    YM = " 01" # Yaw Mode | 00 No control, 01 Mode Speed, 02 Mode Speed Angle
    Rsl = " 00" # Roll speed low byte 
    Rsh = " 00" # Roll speed high byte | 0.1220740379 degree/sec | (2 byte signed, little-endian order)
    Ral = " 00" # Roll angle low byte 
    Rah = " 00" # Roll angle  high byte | 0.02197265625 degree | (2 byte signed, little-endian order)
    Psl = " " + Psl  # Pitch speed low byte 
    Psh = " " + Psh # Pitch speed high byte | 0.1220740379 degree/sec | (2 byte signed, little-endian order)
    Pal = " 00" # Pitch angle low byte 
    Pah = " 00" # Pitch angle  high byte | 0.02197265625 degree | (2 byte signed, little-endian order)
    Ysl = " " + Ysl # Yaw speed low byte 
    Ysh = " " + Ysh # Yaw speed high byte | 0.1220740379 degree/sec | (2 byte signed, little-endian order)
    Yal = " 00" # Yaw angle low byte 
    Yah = " 00" # Yaw angle  high byte | 0.02197265625 degree | (2 byte signed, little-endian order)
    CS =  " {:02x}".format(((int(RM[-2:], 16) + int(PM[-2:], 16) + int(YM[-2:], 16) 
                           + int(Rsl[-2:], 16) + int(Rsh[-2:], 16) + int(Ral[-2:], 16) + int(Rah[-2:], 16) 
                           + int(Psl[-2:], 16) + int(Psh[-2:], 16) + int(Pal[-2:], 16) + int(Pah[-2:], 16) 
                           + int(Ysl[-2:], 16) + int(Ysh[-2:], 16) + int(Yal[-2:], 16) + int(Yah[-2:], 16)) % 256))  # checksum

    command = Header + RM + PM + YM + Rsl + Rsh + Ral + Rah + Psl + Psh + Pal + Pah + Ysl + Ysh + Yal + Yah + CS
    return command

if __name__=="__main__":

    # serial initialization
    Serial = serial.Serial(port='/dev/ttyUSB0',  baudrate=115200, timeout=.1)

    # define the tracker type
    tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'MOSSE', 'CSRT']
    type = 'CSRT'

    # mouse event handler
    cv2.namedWindow("Frame")      # must match the imshow 1st argument
    cv2.setMouseCallback("Frame", click)

    # video capture
    cap = cv2.VideoCapture(0)
    width  = cap.get(cv2.CAP_PROP_FRAME_WIDTH)   # float `width`
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)  # float `height`
    center_frame = (int(width/2), int(height/2)) # center frame coordinate
    
    # command the camera to return to initial position
    for i in range(10):
        command_str = init_pos_command_str
        command_str = bytes.fromhex(command_str) # convert hex to bytes
        Serial.write(command_str)                # send the serial command to camera

    while True:
        # program in define-object mode
        if is_define_object:

            # command the camera to stop 
            command_str = stop_command_str
            command_str = bytes.fromhex(command_str) # convert hex to bytes
            Serial.write(command_str)                # send the serial command to camera

            # tracker initialization
            tracker = tracker_type(type)

            # looping the frames of video camera
            while (is_define_object): 
                stream = cv2.waitKey(1) # Load video every 1ms and to detect user entered key

                # Read from videoCapture stream and display
                ret,frame = cap.read()

                # if no left mouse click yet, the circle will be displayed in (0,0) coordinate
                # if there is left mouse click, the circle will be displayed in the clicked coordinate
                if num_click == 0:
                    cv2.circle(frame, (0,0),radius,colour,lineWidth)     # draw circle in (0,0) coordinate
                else:
                    cv2.circle(frame, point,radius,colour,lineWidth)     # draw circle in clicked coordinate

                cv2.putText(frame, "Define the Object", (int(width/2)-100, 20), # draw text
                                cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

                cv2.imshow("Frame", frame) # displaying the frame

                if stream & 0XFF == ord('q'):  # if 'q' button is pressed, the program is killed
                    is_killing = True
                    break                      # get out of loop
            
            # if there are two left-click events, the program changes to tracking mode
            if len(points) == 2:
                x, y = points[0]                # x and y coordinates of the bounding box's top-left point
                w = points[1][0] - points[0][0] # width of the bounding box
                h = points[1][1] - points[0][1] # height of the bounding box's
                bbox = (x,y,w,h)                # bounding box variables
                
                tracker.init(frame, bbox) # define which ROI the tracker should track
                reference_frame = frame   # ROI frame
                reference_bbox = bbox     # ROI bbox
                print(f'reference_bbox = {reference_bbox}')
                print(f'bbox = {bbox}')

        # program in tracking mode 
        if is_tracking:
            # looping the frames of video camera
            while (is_tracking):
                stream = cv2.waitKey(1)   # Load video every 1ms and to detect user entered key
                
                # Read from videoCapture stream and display
                ret,frame = cap.read()

                timer = cv2.getTickCount()
                ret, bbox = tracker.update(frame) # update the tracker with new frames
                fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer) # count the fps
                dt = 1/fps

                # get the difference between frames in pixels for failsafe 
                # if the tracker's still detecting when the object is lost
                frame_diff_value = frame_diff(reference_frame, frame)

                # if there is a frame and the differences with the frame before is less than the threshold
                # do the camera angular velocity calculation
                if ret and frame_diff_value < failure_threshold:
                    p1 = (int(bbox[0]), int(bbox[1])) # the top-left point coordinate
                    p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3])) # the bottom-right point coordinate
                    center_bbox = (int(bbox[0] + bbox[2]//2), int(bbox[1] + bbox[3]//2)) # the object's bounding box center coordinate

                    cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1) # draw bounding box
                    cv2.circle(frame, center_bbox, radius, colour, lineWidth) # draw point of center of bounding box
                    cv2.circle(frame, center_frame, radius, (255,255,0), lineWidth) # draw point of center of frame

                    # calculate camera yaw and pitch velocity
                    yaw = calc_vel_PID_yaw(max, min, center_bbox[0], center_frame[0], 0.09, 0.000, 0.016, dt)
                    pitch = calc_vel_PID_pitch(max, min, center_bbox[1], center_frame[1], 0.09, 0.000, 0.016, dt)

                    hex_vels = vel2hex(yaw, pitch) # change yaw and pitch velocity to hex format
                    command_str = control_parser(hex_vels[0], hex_vels[1], hex_vels[2], hex_vels[3]) # creating serial protocol command
                    command_str = bytes.fromhex(command_str) # convert hex to bytes
                    
                    print(f'yaw = {yaw}, pitch = {pitch}')

                # if there is no frame or the differences with the frame before is more than the threshold
                else:
                    cv2.putText(frame, "Tracking failure detected", (100,80), # draw text
                                cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
                    
                    # reinitiate the tracker
                    tracker = tracker_type(type)
                    tracker.init(reference_frame, reference_bbox)
                    # tell the camera to stop moving
                    command_str = bytes.fromhex(stop_command_str)
                    
                cv2.putText(frame, type + " Tracker", (100,20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2) # draw text
                cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2) # draw text

                Serial.write(command_str)  # send the serial command to the camera
                cv2.imshow("Frame", frame) # displaying the frame 
            
                if stream & 0XFF == ord('q'):  # if 'q' button is pressed, the program is killed
                    is_killing = True
                    break                      # get out of loop

        if is_killing:  # kill program
            break                      

    # after the program is killed, tell the camera to stop moving  
    command_str = bytes.fromhex(stop_command_str)
    Serial.write(command_str)
    print("Tracking done")

    # closing serial comms, stop receiving frame, kill all displays
    Serial.close()
    cap.release()
    cv2.destroyAllWindows()
