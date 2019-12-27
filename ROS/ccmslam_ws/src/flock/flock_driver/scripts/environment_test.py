import sys
import traceback
# import av
import cv2
import numpy
import tellopy
import libh264decoder


# Test the environment: TelloPy + PyAV + NumPy + OpenCV + all connections
# If everything is working you should see a cv2 window with 6x6 ArUco markers highlighted in green

def main():
    drone = tellopy.Tello()

    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters_create()

    try:
        # Connect to drone
        drone.connect()
        drone.wait_for_connection(60.0)

        # Get video stream
        # container = av.open(drone.get_video_stream())
        container = 



        for frame in container.decode(video=0):

            # PyAV frame => PIL image => OpenCV Mat
            color = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)

            # Color => gray for detection
            gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

            # Detect markers
            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            # Draw border on the color image
            color = cv2.aruco.drawDetectedMarkers(color, corners)
            cv2.imshow('Markers', color)
            cv2.waitKey(1)
            #print(corners)

    except Exception as ex:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(ex)
    finally:
        drone.quit()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
