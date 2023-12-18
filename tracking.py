import serial
import cv2
import numpy as np

# Constant
colour = (0,255,0)
lineWidth = -1      # -1 will result in filled circle
radius = 3
point = (0,0)
points = []
num_click = 0
is_define_object = True
is_tracking = False
is_killing = False
failure_threshold = 100
integral_x = 0
pre_error_x = 0
integral_y = 0
pre_error_y = 0
min = -0.123 * 20 # degree/s
max = 0.123 * 20  # degree/s
reference_frame = 0
reference_bbox = (0, 0, 20, 20) # just initialize
base_vel = 0.1220740379
stop_command_str = "FF 01 0F 10 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00"
init_pos_command_str = "FF 01 0F 10 02 02 02 00 00 00 00 00 00 00 00 00 00 00 00 06"

def tracker_type(type):
    # tracker
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
 
# function for detecting left mouse click
def click(event, x,y, flags, param):
    global point, num_click, points, is_define_object, is_tracking

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


    if event == cv2.EVENT_RBUTTONDOWN:
        num_click = 0
        points = []
        is_define_object = True
        is_tracking = False

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

def to_grayscale(arr):
    "If arr is a color image (3D array), convert it to grayscale (2D array)."
    if len(arr.shape) == 3:
        return np.mean(arr, -1)  
    else:
        return arr

def normalize(arr):
    rng = arr.max()-arr.min()
    amin = arr.min()
    return (arr-amin)*255/rng

def frame_diff(frame_bgr_ref, frame_bgr_target):
    frame_bgr_ref = to_grayscale(frame_bgr_ref.astype(float))
    frame_bgr_target = to_grayscale(frame_bgr_target.astype(float))

    diff = frame_bgr_target - frame_bgr_ref  # elementwise for scipy arrays
    m_norm = np.sum(abs(diff))  # Manhattan norm
    return m_norm * 1.0 / frame_bgr_ref.size


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

    # serial init
    Serial = serial.Serial(port='/dev/ttyUSB0',  baudrate=115200, timeout=.1)

    # tracking init
    tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'MOSSE', 'CSRT']
    type = 'CSRT'

    # event handler
    cv2.namedWindow("Frame")      # must match the imshow 1st argument
    cv2.setMouseCallback("Frame", click)

    # video capture
    cap = cv2.VideoCapture(0)
    width  = cap.get(cv2.CAP_PROP_FRAME_WIDTH)   # float `width`
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)  # float `height`
    center_frame = (int(width/2), int(height/2))
    
    for i in range(10):
        command_str = init_pos_command_str
        command_str = bytes.fromhex(command_str)
        Serial.write(command_str)

    while True:
        if is_define_object:
            command_str = stop_command_str
            command_str = bytes.fromhex(command_str)
            Serial.write(command_str)

            tracker = tracker_type(type)

            while (is_define_object): 
                stream = cv2.waitKey(1) # Load video every 1ms and to detect user entered key

                # Read from videoCapture stream and display
                ret,frame = cap.read()

                # frame_corr_value = frame_corr(frame, frame)
                # print(frame_corr_value)

                if num_click == 0:
                    cv2.circle(frame, (0,0),radius,colour,lineWidth)     # circle properties as arguments
                else:
                    cv2.circle(frame, point,radius,colour,lineWidth)     # circle properties as arguments

                cv2.putText(frame, "Define the Object", (int(width/2)-100, 20), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

                cv2.imshow("Frame", frame)

                if stream & 0XFF == ord('q'):  # If statement to stop loop,Letter 'q' is the escape key
                    is_killing = True
                    break                      # get out of loop
            
            if len(points) == 2:
                x, y = points[0]
                w = points[1][0] - points[0][0]
                h = points[1][1] - points[0][1]
                bbox = (x,y,w,h)
                
                tracker.init(frame, bbox)
                reference_frame = frame
                reference_bbox = bbox
                print(f'reference_bbox = {reference_bbox}')
                print(f'bbox = {bbox}')

        if is_tracking:
            #Loop for video stream
            while (is_tracking):
                stream = cv2.waitKey(1)   # Load video every 1ms and to detect user entered key
                
                # Read from videoCapture stream and display
                ret,frame = cap.read()

                timer = cv2.getTickCount()
                ret, bbox = tracker.update(frame)
                fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
                dt = 1/fps

                frame_diff_value = frame_diff(reference_frame, frame)

                if ret and frame_diff_value < failure_threshold:
                    p1 = (int(bbox[0]), int(bbox[1]))
                    p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                    center_bbox = (int(bbox[0] + bbox[2]//2), int(bbox[1] + bbox[3]//2))

                    cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
                    cv2.circle(frame, center_bbox, radius, colour, lineWidth)
                    cv2.circle(frame, center_frame, radius, (255,255,0), lineWidth)

                    yaw = calc_vel_PID_yaw(max, min, center_bbox[0], center_frame[0], 0.1, 0.000, 0.01, dt)
                    pitch = calc_vel_PID_pitch(max, min, center_bbox[1], center_frame[1], 0.1, 0.000, 0.01, dt)

                    hex_vels = vel2hex(yaw, pitch)
                    command_str = control_parser(hex_vels[0], hex_vels[1], hex_vels[2], hex_vels[3])
                    #print(command_str)
                    command_str = bytes.fromhex(command_str)
                    
                    print(f'yaw = {yaw}, pitch = {pitch}')

                else:
                    cv2.putText(frame, "Tracking failure detected", (100,80), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
                    tracker = tracker_type(type)
                    tracker.init(reference_frame, reference_bbox)
                    command_str = bytes.fromhex(stop_command_str)
                    
                cv2.putText(frame, type + " Tracker", (100,20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)
                cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)

                Serial.write(command_str)
                cv2.imshow("Frame", frame)
            
                if stream & 0XFF == ord('q'):  # If statement to stop loop,Letter 'q' is the escape key
                    is_killing = True
                    break                      # get out of loop

        if is_killing:  # kill program
            break                      
        
    command_str = bytes.fromhex(stop_command_str)
    Serial.write(command_str)
    print("Tracking done")

    Serial.close()
    cap.release()
    cv2.destroyAllWindows()
