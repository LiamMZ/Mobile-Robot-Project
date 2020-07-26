import cv2
import argparse
import numpy as np
import rospy
import time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

# ap = argparse.ArgumentParser()
# ap.add_argument('-c', '--config', required=True,
#                 help = 'path to yolo config file')
# ap.add_argument('-w', '--weights', required=True,
#                 help = 'path to yolo pre-trained weights')
# ap.add_argument('-cl', '--classes', required=True,
#                 help = 'path to text file containing class names')
# args = ap.parse_args()

def get_output_layers(net):
    layer_names = net.getLayerNames()
    #gets layer names of unconnected output layers
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

    return output_layers


def draw_prediction(img, class_id, confidence, x, y, x_plus_w, y_plus_h, COLORS, classes):

    label = str(classes[class_id])

    color = COLORS[class_id]
    color = np.array([int(c) for c in color])
    cv2.rectangle(img, (int(x),int(y)), (int(x_plus_w),int(y_plus_h)), color)

    cv2.putText(img, label, (int(x)-10,int(y)-10), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
    print(label)

def processFrame(image, args):
    Width = image.shape[1]
    Height = image.shape[0]
    scale = 0.00392
    classes = None
    print("here5")
    #get classes from classes command line argument 
    with open(args.classes, 'r') as f:
        classes = [line.strip() for line in f.readlines()]
    COLORS = np.random.uniform(0, 255, size=(len(classes), 3))

    #build net using weights argument and net conficuration argument
    net = cv2.dnn.readNet(args.weights, args.config)

    #convert image to 4 dimensional blob, scale by scale-factor
    blob = cv2.dnn.blobFromImage(image, scale, (416,416), (0,0,0), True, crop=False)

    #set new input values
    net.setInput(blob)

    #computes output layer values based on names of output layers
    outs = net.forward(get_output_layers(net))
    
    class_ids = []
    confidences = []
    boxes = []
    conf_threshold = 0.5
    nms_threshold = 0.4
    #loop through output layer values
    for out in outs:
        #loops through detections
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:
                #print(detection)
                center_x = int(detection[0] * Width)
                center_y = int(detection[1] * Height)
                w = int(detection[2] * Width)
                h = int(detection[3] * Height)
                x = center_x - w / 2
                y = center_y - h / 2
                class_ids.append(class_id)
                confidences.append(float(confidence))
                boxes.append([x, y, w, h])


    indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)

    for i in indices:
        i = i[0]
        box = boxes[i]
        x = box[0]
        y = box[1]
        w = box[2]
        h = box[3]
        draw_prediction(image, class_ids[i], confidences[i], round(x), round(y), round(x+w), round(y+h), COLORS, classes)
    return image

cv_image = None

def callback(ros_data):
    global cv_image
    # bridge = CvBridge()
    # cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
     #### direct conversion to CV2 ####
    np_arr = np.fromstring(ros_data.data, np.uint8)
    # image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
    cv_image = image_np
    # cv2.imshow('cv_img', cv_image)
    # cv2.waitKey(1)
    


class Args:
    def __init__(self):
        pass

def listener():
    pass
    

def main():        
    rospy.init_node('yolo_webcam')
    global cv_image
    img_sub = rospy.Subscriber("/phone1/camera/image/compressed", CompressedImage, callback)

    args = Args()
    args.classes = 'yolov3.txt'
    args.weights = 'yolov3.weights'
    args.config = 'yolov3.cfg'
    print("here3")

    while not rospy.is_shutdown():
        print("here4")
        frame = cv_image
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if frame is not None:    
            processed = processFrame(frame, args)
            cv2.imshow('processed', frame)

        #sets the wait key to 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    print(cv2. __version__)
    main()