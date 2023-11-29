import cv2
import apriltag_pybind as apriltag

# Load image
img = cv2.imread("aprilgrid.jpeg", cv2.IMREAD_GRAYSCALE)
tag_data = apriltag.detect(img)

# Detect
viz = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
for (tag_id, corner_idx, kp_x, kp_y) in tag_data:
    pt = (int(kp_x), int(kp_y))
    radius = 5
    color = (0, 0, 255)
    thickness = 2
    cv2.circle(viz, pt, radius, color, thickness)

# Visualize
cv2.imshow("Viz", viz)
cv2.waitKey(0)
