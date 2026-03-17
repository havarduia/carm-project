import cv2
import cv2.aruco as aruco

# board layout
markersX = 3
markersY = 4

# sizes in pixels
markerLength = 400
markerSeparation = 120

dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)

board = aruco.GridBoard(
    (markersX, markersY),
    markerLength,
    markerSeparation,
    dictionary
)

# generate image
img = board.generateImage((2000, 2800), marginSize=50)

cv2.imwrite("aruco_board.png", img)

print("Saved aruco_board.png")