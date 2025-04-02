import cv2
import numpy as np

def draw_circle(num_nodes=50):
    # Define image size
    height, width = 2048, 2448
    
    # Create a black image
    img = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Define circle parameters
    center = (width // 2, height // 2)
    radius = min(width, height) // 3  # Adjust circle size relative to window
    
    # Generate circle points
    angles = np.linspace(0, 2 * np.pi, num_nodes, endpoint=False)
    for angle in angles:
        x = int(center[0] + radius * np.cos(angle))
        y = int(center[1] + radius * np.sin(angle))
        cv2.circle(img, (x, y), 3, (0, 0, 255), -1)  # Draw red nodes
    
    # Display the image
    cv2.imshow("Red Circle", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Adjust the number of nodes here
draw_circle(num_nodes=100)
