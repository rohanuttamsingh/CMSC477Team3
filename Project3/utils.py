image_width = 1280
threshold = 5
detect_every_n_frames = 5

def detect(model, image):
    return model.predict(image, confidence=50, overlap=30).json()['predictions']

def can_see_lego(predictions):
    for prediction in predictions:
        if prediction['class'] == 'lego':
            return True
    return False

def get_lego_coords(predictions):
    for prediction in predictions:
        if prediction['class'] == 'lego':
            return prediction['x'], prediction['y']
    return None

def can_see_robot(predictions):
    for prediction in predictions:
        if prediction['class'] == 'robot':
            return True
    return False

def get_robot_coords(predictions):
    for prediction in predictions:
        if prediction['class'] == 'robot':
            return prediction['x'], prediction['y']
    return None
