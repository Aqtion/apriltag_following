from dt_apriltags import Detector
import cv2
import matplotlib.pyplot as plt
import numpy as np
from pid import PID

def process(frame, pid_x, pid_y):
    dimensions = frame.shape
    height = dimensions[0]/2
    width = dimensions[1]/2
    at_detector = Detector(families='tag36h11',
                    nthreads=1,
                    quad_decimate=1.0,
                    quad_sigma=0.0,
                    refine_edges=1,
                    decode_sharpening=0.25,
                    debug=0)
    cameraMatrix = np.array([ 1060.71, 0, 960, 0, 1060.71, 540, 0, 0, 1]).reshape((3,3))
    width = 960
    height = 540
    camera_params = ( cameraMatrix[0,0], cameraMatrix[1,1], cameraMatrix[0,2], cameraMatrix[1,2] )
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(gray, estimate_tag_pose=True, camera_params = camera_params, tag_size=0.1)
    color_img = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
    crosshair_offset = 40
    cv2.line(color_img, (int(width - crosshair_offset), int(height)), (int(width + crosshair_offset), int(height)), (255,0,0), 5)
    cv2.line(color_img, (int(width), int(height - crosshair_offset)), (int(width), int(height + crosshair_offset)), (255,0,0), 5)
    # draw_tags(color_img, tags)
    # plt.imshow(color_img)
    try:
        # print(tags)
        x_error = 0
        y_error = 0
        center_x = 0
        center_y = 0
        for tag in tags:
            for idx in range(len(tag.corners)):
                cv2.line(color_img, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0), 5)
                translation_matrix = tag.pose_t.reshape(1,3)
                print(translation_matrix)
                
                x = translation_matrix[0][0]
                y = translation_matrix[0][1]
                z = translation_matrix[0][2]

                x_error += x
                y_error += y
                # output_video.write(color_img)
            center_x += tag.center[0]
            center_y += tag.center[1]



        center_x = center_x / len(tags)
        center_y = center_y / len(tags)
        print("x_err", center_x)
        print("y_err", center_y)
        
        crosshair_offset = 20
        cv2.line(color_img, (int(center_x-crosshair_offset), int(center_y)), (int(center_x+crosshair_offset), int(center_y)), (0,0,255), 5)
        cv2.line(color_img, (int(center_x), int(center_y-crosshair_offset)), (int(center_x), int(center_y+crosshair_offset)), (0,0,255), 5)

        cv2.line(color_img, (int(width), int(height)), (int(center_x), int(center_y)), (255,0,0), 5)
        avg_x_error = x_error / len(tags)
        avg_y_error = y_error / len(tags)
        x_output = pid_x.update(avg_x_error)
        y_output = pid_y.update(avg_y_error)
        print("x: ", x_output)
        print("y: ", y_output)

        str_x_out = "x_output: " + str(x_output)
        str_y_out = "y_output: " + str(y_output)

        cv2.putText(color_img, str_x_out, (int(100),int(100)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 5, cv2.LINE_AA)

        
        cv2.putText(color_img, str_y_out, (int(1320),int(980)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 5, cv2.LINE_AA)

        
        p_img = color_img.copy()
        return [x_output, y_output]
    except:
        pass
        
    