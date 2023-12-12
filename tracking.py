import numpy as np
import cv2

# Constant
colour = (0,255,0)
lineWidth = -1      # -1 will result in filled circle
radius = 3
point = (0,0)
points = []
num_click = 0
first_detected = 0
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

if __name__=="__main__":
    tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'MOSSE', 'CSRT']
    type = 'CSRT'
    tracker = tracker_type(type)

    # event handler
    cv2.namedWindow("Frame")      # must match the imshow 1st argument
    cv2.setMouseCallback("Frame", click)
    cap = cv2.VideoCapture(0)

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
                if ret:
                    p1 = (int(bbox[0]), int(bbox[1]))
                    p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                    cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
                    center_point = (int(bbox[0] + bbox[2]//2), int(bbox[1] + bbox[3]//2))
                    cv2.circle(frame, center_point, radius, colour, lineWidth)
                    print(f'({center_point[0]}, {center_point[1]})')
                
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
