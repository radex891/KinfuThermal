import numpy as np
import myfunctions as mf
from numpy import load
import sys, getopt

def main(argv):
    try:
        opts, args = getopt.getopt(argv, "hi:o:", "ifile=")
    except getopt.GetoptError:
        print('offsetBetweenBags.py -i <my_calibration_bags>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('\n----get offset information----\n'
                   '----camera matrix for initial for thermal image rectification cx cy fx fy \n'
                  '----distortion parameters for thermal image rectification d1 d2 d3 d4 d5 \n'
                  'offsetBetweenBags.py -i <offset_bag> '
                  'cx=<float> cy=<float> fx=<float> fy=<float> d1=<float> d2<float> d3<float> d4<float> d5<float>\n')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            bagfile = arg
    path_and_name = str(bagfile)
    if path_and_name[0] != '/':
        path_and_name = '/' + path_and_name
    cx, cy, fx, fy = 194.37857456, 144.26050569, 374.47524472, 374.48175626
    d1, d2, d3, d4, d5 = -0.42921441, 0.26279496, -0.00111263, 0.00098839, -0.10130903
    if len(args) != 0:
        print('args....................................', args)
        for arg in args:
            if arg[:2] == 'd1':
                d1 = float(arg[3:])
                print('arg..................', d1)
            if arg[:2] == 'd2':
                d2 = float(arg[3:])
                print('arg..................', d2)
            if arg[:2] == 'd3':
                d3 = float(arg[3:])
                print('arg..................', d3)
            if arg[:2] == 'd4':
                d4 = float(arg[3:])
                print('arg..................', d4)
            if arg[:2] == 'd5':
                d5 = float(arg[3:])
                print('arg..................', d5)
            if arg[:2] == 'cx':
                cx = float(arg[3:])
                print('arg..................', cx)
            if arg[:2] == 'cy':
                cy = float(arg[3:])
                print('arg..................', cy)
            if arg[:2] == 'fx':
                fx = float(arg[3:])
                print('arg..................', fx)
            if arg[:2] == 'fy':
                fy = float(arg[3:])
                print('arg..................', fy)
    topic1 = '/optris/image_default'
    topic2 = '/camera/depth/image_rect_raw'
    topic4 = '/optris/camera_info'
    topic5 = '/camera/depth/camera_info'
    topic6 = '/optris/flag_state'
    topics_in_bag = topic1, topic2, topic4, topic5, topic6
    camera_matrix = np.array([[fx, 0., cx],
                              [0., fy, cy],
                              [0., 0., 1.]])
    distortion_params = np.array([[d1, d2, d3, d4, d5]])
    path_still_images, path_optris, path_rs = mf.write_images_from_rosbag_to_folder2(path_and_name, camera_matrix, distortion_params, topics_in_bag)
    mf.delete_donotuse(path_optris)
    mask_optris = (100, 0, 200, 210)     # masked pixels: left, right, top, bottom
    mask_rs = (100, 100, 200, 0)           # masked pixels: left, right, top, bottom
    optris_binary_threshold = 79
    depth_range = (2000, 3000)
    mf.track_movement(path_optris, path_rs, False, False, optris_binary_threshold, depth_range, mask_optris, mask_rs)

    rs_XY_array = load(path_and_name[:-4] + '/array_cache/rs_XY_array.npy')
    op_XY_array = load(path_and_name[:-4] + '/array_cache/optris_XY_array.npy')

    intervals = np.zeros((4, 2))
    intervals[0, :] = [50, 120]                                # additional time offset
    intervals[1, :] = [0, 20]                                # Y-offset
    intervals[2, :] = [0.6, 1]                            # Y-scale
    intervals[3, :] = [0.994, 0.997]                         # drift factor
    loops = 1000
    rand_params = mf.random_params(intervals, loops)
    rand_params[0, :].astype(int)
    new_params = rand_params[:, 0]
    error_old = 999999
    index = 0
    fraction = 1/3
    for j in range(4):
        print('Interval length:', np.round((2 * fraction) ** j, 3))
        if j > 0:
            intervals = mf.decrease_interval(intervals, rand_params[:, index], fraction)
            rand_params = mf.random_params(intervals, loops)
            rand_params[0, :].astype(int)

        for i in range(len(rand_params[0, :])):
            error = mf.error_function_offset(rand_params[:, i], rs_XY_array, op_XY_array)
            if error < error_old:
                error_old = error
                index = i
                new_params = rand_params[:, i]
                print('             new best params:', new_params, 'RMSE:', np.round(error_old, 4), 'Iteration:', i)

    params = new_params
    x1, y1 = mf.spline_interpolation_rs(rs_XY_array)
    x2, y2 = mf.spline_interpolation_op_wo_calibration_pause(params, op_XY_array)
    mf.plot_time_difference(x1, y1, x2, y2, path_and_name, params[0]/10, params[3])
    x1, y1, x2, y2 = mf.truncate_data(params, rs_XY_array, op_XY_array)
    mf.plot_time_difference(x1-x1[0], y1, x2-x1[0], y2, path_and_name, 0, 0)

    df_rs = np.column_stack((x1-x1[0], y1))
    df_op = np.column_stack((x2-x1[0], y2))

if __name__ == "__main__":
    main(sys.argv[1:])




