"""
import cv2

# recognize the image
car_image = 'big back.jpg'

# classifier
classifier = 'cars.xml'

# import image
img = cv2.imread(car_image)

# smaller img size
#min_img = cv2.resize(img, (600, 600))

# cvt to BnW
BnW = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# clasify the car in the image
# without import cv2, wont have Cascade arguement
car_tracker = cv2.CascadeClassifier(classifier)

# detect cars
# multiscale = detect car whether big or small, it will concert cars into coordiante
cars = car_tracker.detectMultiScale(BnW)

for (x, y, w, h) in cars:
    cv2.rectangle(img, (x, y),((x+w),(y+h)),(0, 0, 255), 2)

# display image
cv2.imshow('yapzihwa', img)

cv2.waitKey()

print("code completed")
"""

import cv2

video = cv2.VideoCapture('people car.mp4')

car_classifier = 'cars.xml'
pedestrian_classifier = 'full body.xml'

# car and people classifier
car_tracker = cv2.CascadeClassifier(car_classifier)
pedestrian_tracker = cv2.CascadeClassifier(pedestrian_classifier)

while True:
    (read_successful, frame) = video.read()

#   if read_successful:
#      grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    cars = car_tracker.detectMultiScale(frame)
    pedestrian = pedestrian_tracker.detectMultiScale(frame)

    print(cars)
    print(pedestrian)

    for (x, y, w, h) in cars:
        cv2.rectangle(frame, (x, y), ((x + w), (y + h)), (0, 0, 255), 2)

    for (x, y, w, h) in pedestrian:
        cv2.rectangle(frame, (x, y), ((x + w), (y + h)), (0, 255, 255), 2)

    cv2.imshow('finally', frame)
    key = cv2.waitKey(10)

    if key == 81 or key == 113:
        break

video.release()