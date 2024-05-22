from GUI import GUI
from HAL import HAL

import cv2
import numpy as np
import tensorflow as tf
print(tf.__version__)

model = tf.keras.models.load_model('/data/models/best_model_trained.h5')


def preprocess_image(image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    image = cv2.resize(image, (200, 200))
    image = image / 255.0
    image = np.expand_dims(image, axis=0)
    return image

def predict_speed(image_path):
    image = preprocess_image(image_path)
    prediction = model.predict(image)
    linear_speed, angular_speed = prediction[0]
    return linear_speed, angular_speed



while True:
    image = HAL.getImage()
    linear_speed, angular_speed = predict_speed(image)
    print("Predicted Linear Speed:", linear_speed)
    print("Predicted Angular Speed:", angular_speed)
    HAL.setV(linear_speed)
    HAL.setW(angular_speed)