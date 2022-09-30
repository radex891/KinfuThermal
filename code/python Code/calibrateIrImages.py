import numpy as np
import myfunctions as mf
import cv2
import sys, getopt

# # mtx = [[369.87246895,   0.,         195.47222186],
# #  [  0.,         369.64354162, 154.22475512],
# #  [  0.,           0.,           1.        ]]
#path_and_name = '/home/k/Dokumente/my_calibration_bags/'

def main(argv):
    try:
        opts, args = getopt.getopt(argv, "hi:o:", "ifile=")
    except getopt.GetoptError:
        print('calibrateIrImages.py -i <my_calibration_bags>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('\n----provide initial guess----\n '
                  '\n----sequence of arguments is mandatory----\n '
                  'calibrateIrImages.py -i <my_calibration_bags> <cx> <cy> <fx> <fy>\n')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            bagfile = arg
    path_and_name = str(bagfile)
    if path_and_name[0] != '/':
        path_and_name = '/' + path_and_name
    print('Patterns are being loaded........')
    objpoints, imgpoints, shape = mf.load_pattern_coordinates_from_ir(path_and_name, load_calibrated_points=False,
                                                                      show_patterns=False)
    if len(args) == 4:
        cx = args[0]
        cy = args[1]
        fx = args[2]
        fy = args[3]
        initial_guess = np.array([[fx,  0, cx],
                                [  0, fy, cy],
                                [  0,   0,   1]]).astype(float)
        print('initial guess for camera matrix:\n', initial_guess)
        print('calibrating now............................')
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, shape, initial_guess, None, flags=(cv2.CALIB_USE_INTRINSIC_GUESS))
    if len(args) == 0:
        print('calibrating now............................')
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, shape, None, None)
    print('updated camera matrix\n', mtx)
    print('calculated distortion parameters\n', dist)
    np.savetxt(path_and_name + 'mtx.csv', mtx, delimiter=",")
    np.savetxt(path_and_name + 'dist.csv', dist, delimiter=",")
    mf.undistort_images(path_and_name, mtx, dist)
    print('calibration process finished.....................................')
    #np.loadtxt(path_and_name.rsplit('/', 1)[0] + '/camera_matrix_optris.csv', delimiter=",")


if __name__ == "__main__":
    main(sys.argv[1:])