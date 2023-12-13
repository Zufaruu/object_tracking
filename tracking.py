import numpy as np
import cv2

# Constant
colour = (0,255,0)
lineWidth = -1      # -1 will result in filled circle
radius = 3
point = (0,0)
points = []
num_click = 0
integral_x = 0
pre_error_x = 0
integral_y = 0
pre_error_y = 0
min = -15
max = 15
reference_frame = 0
reference_bbox = (0, 0, 20, 20) # just initialize

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
    global point, num_click, points
    if event == cv2.EVENT_LBUTTONDOWN and num_click < 2:
        num_click = num_click + 1
        point = (x,y)
        points.append(point)
        print(f'num_click = {num_click}')
        print(f"coordinate = ({x}, {y})")
        print(f'points {points}')

def calc_vel_PID_yaw(x_max, x_min, x_setpoint, x_pv, kp, ki, kd, dt):
    global integral_x, pre_error_x
    error = x_setpoint - x_pv

    Pout = kp * error

    integral_x = integral_x + (error * dt)
    Iout = ki * integral_x

    derivative_x = (error - pre_error_x) / dt
    Dout = kd * derivative_x
    pre_error_x = error

    vel_x = Pout + Iout + Dout

    if output > x_max:
        output = x_max
    elif output < x_min:
        output = x_min
    
    return vel_x

def calc_vel_PID_pitch(y_may, y_min, y_setpoint, y_pv, kp, ki, kd, dt):
    global integral_y, pre_error_y
    error = y_setpoint - y_pv

    Pout = kp * error

    integral_y = integral_y + (error * dt)
    Iout = ki * integral_y

    derivative_y = (error - pre_error_y) / dt
    Dout = kd * derivative_y
    pre_error_y = error

    vel_y = Pout + Iout + Dout

    if output > y_may:
        output = y_may
    elif output < y_min:
        output = y_min
    
    return vel_y

if __name__=="__main__":

    # tracking init
    tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'MOSSE', 'CSRT']
    type = 'CSRT'
    tracker = tracker_type(type)

    # event handler
    cv2.namedWindow("Frame")      # must match the imshow 1st argument
    cv2.setMouseCallback("Frame", click)

    # video capture
    cap = cv2.VideoCapture(0)
    width  = cap.get(cv2.CAP_PROP_FRAME_WIDTH)   # float `width`
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)  # float `height`
    center_frame = (int(width/2), int(height/2))

    if len(points) < 2:
        #Loop for video stream
        while (len(points) < 2): 
            stream = cv2.waitKey(1)   # Load video every 1ms and to detect user entered key
            
            # Read from videoCapture stream and display
            ret,frame = cap.read()
            cv2.circle(frame, point,radius,colour,lineWidth)     # circle properties as arguments

            cv2.imshow("Frame", frame)

            if stream & 0XFF == ord('q'):  # If statement to stop loop,Letter 'q' is the escape key
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


    if len(points) == 2:
        #Loop for video stream
        while (True):
            stream = cv2.waitKey(1)   # Load video every 1ms and to detect user entered key
            
            # Read from videoCapture stream and display
            ret,frame = cap.read()

            # main code
            if len(points) == 2:
                # cv2.rectangle(frame, points[0], points[1], (255, 0, 0), 2) 

                timer = cv2.getTickCount()
                ret, bbox = tracker.update(frame)
                fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
                dt = 1/fps

                if ret:
                    p1 = (int(bbox[0]), int(bbox[1]))
                    p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                    center_bbox = (int(bbox[0] + bbox[2]//2), int(bbox[1] + bbox[3]//2))

                    cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
                    cv2.circle(frame, center_bbox, radius, colour, lineWidth)
                    cv2.circle(frame, center_frame, radius, (255,255,0), lineWidth)

                    yaw = calc_vel_PID_yaw(max, min, center_bbox[0], center_frame[0], 1, 0, 0, dt)
                    pitch = calc_vel_PID_pitch(max, min, center_bbox[1], center_frame[1], 1, 0, 0, dt)

                    print(f'yaw = {yaw}, pitc')
                
                else:
                    cv2.putText(frame, "Tracking failure detected", (100,80), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
                    tracker = tracker_type(type)
                    tracker.init(reference_frame, reference_bbox)
                    
                cv2.putText(frame, type + " Tracker", (100,20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)
                cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)

            cv2.imshow("Frame", frame)
            # print(f'reference_bbox = {reference_bbox}, reference_frame = {reference_frame[0]}')
            
            if stream & 0XFF == ord('q'):  # If statement to stop loop,Letter 'q' is the escape key
                break                      # get out of loop
            
    cap.release()
    cv2.destroyAllWindows()
