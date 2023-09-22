import cv2
import pyzbar.pyzbar as pyzbar
import os
import rospy
from std_msgs.msg import Bool
from rospy import Publisher


def on_board_led(val):
    os.system(f'sudo sh -c "echo {val} > /sys/class/leds/led0/brightness"')
    

rospy.init_node("vision_node", anonymous=True)
pub = Publisher('/vision_topic', Bool, queue_size=10)
qr_expected = ( "MobileApp" )

# Open the front camera of the raspbery pi
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect QR codes in the grayscale frame
    qr_codes = pyzbar.decode(gray)
    
    if not len(qr_codes):
        on_board_led(val=0)
        pub.publish(Bool(False))

    # Loop through the detected QR codes
    for qr in qr_codes:
        # Extract the QR code data as string
        qr_data = qr.data.decode("utf-8")

        qr_start_point = ( qr.rect.left, qr.rect.top )
        qr_end_point = ( qr_start_point[0] + qr.rect.width, qr_start_point[1] + qr.rect.height )
        
        cv2.rectangle(frame, qr_start_point, qr_end_point, color=(255, 0, 0), thickness=2)
        if qr_data in qr_expected:
            on_board_led(val=1)
            pub.publish(Bool(True))
        else:
            print("ok..")
            on_board_led(val=0)
            pub.publish(Bool(False))
        

    # Display the frame with QR code detection
    cv2.imshow('QR Code Detection', frame)

    # Exit the loop when 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
