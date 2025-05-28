import cv2
import numpy as np

# Example bounding boxes (x_center, y_center, width, height)
bbox_data = np.array([
    [477.9799, 549.7693, 288.7865, 177.4233],
    [1062.1976, 460.5398, 54.8376, 41.2450],
    [986.1252, 467.1465, 65.1151, 49.9045],
    [663.1533, 484.1956, 92.9199, 57.7829],
    [836.7966, 469.1197, 66.8079, 52.4786],
    [886.4005, 443.5556, 34.4533, 27.3412],
    [711.0706, 459.7118, 93.0200, 42.8055],
    [592.7981, 471.9813, 57.2784, 45.4734],
    [1047.8896, 437.9255, 29.2740, 25.5943],
    [769.0712, 459.5161, 49.4385, 30.7137],
    [909.7103, 423.7793, 23.9183, 21.5485]
])



# Create a blank image (white background)
#image = np.ones((720, 1280, 3), dtype=np.uint8) * 255  # 720p resolution with 3 channels (RGB)

# Path to the image
image_path = "/home/wise/Documents/sjy/v1.0-mini/samples/CAM_FRONT/n008-2018-08-01-15-16-36-0400__CAM_FRONT__1533151605512404.jpg"

# Load the image
image = cv2.imread(image_path)

# Convert to (x_min, y_min, x_max, y_max)
for bbox in bbox_data:
    x_center, y_center, width, height = bbox
    x_min = int(x_center - width / 2)
    x_max = int(x_center + width / 2)
    y_min = int(y_center - height / 2)
    y_max = int(y_center + height / 2)
    
    # Draw rectangle on the image (color = red, thickness = 2)
    cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (0, 0, 255), 2)

# Display the image
cv2.imshow('Bounding Boxes', image)

# Wait for a key press and close the window
cv2.waitKey(0)
cv2.destroyAllWindows()

