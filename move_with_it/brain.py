import cv2 as cv
import mediapipe as mp

""" Importing the hands module """
mp_hands = mp.solutions.hands

""" Creating an object of the hands class """
hands = mp_hands.Hands(
    model_complexity=0,
    max_num_hands=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

def movement(image):
    """ Flipping the image for a selfie-view display """
    image = cv.flip(image, 1) 
    
    """ To improve performance, mark the image as not writeable. """
    image.flags.writeable = False
    image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
    results = hands.process(image)
    
    image.flags.writeable = True
    image = cv.cvtColor(image, cv.COLOR_RGB2BGR)
    
    if results.multi_hand_landmarks not in (None, []):
        for hand_landmark in results.multi_hand_landmarks:
            
            """ Storing the coordinates of the wrist """
            index_tip = hand_landmark.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
            
            return [index_tip.x, index_tip.y]
        
    return [0, 0]
