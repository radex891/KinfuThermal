import myfunctions as mf
import numpy as np
import scipy
from scipy.optimize import minimize
import sys, getopt

def main(argv):
    try:
        opts, args = getopt.getopt(argv, "hi:o:", "ifile=")
    except getopt.GetoptError:
        print('calibrateCameras.py -i <my_calibration_bags>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('\n----calibrate cameras----\n'
                  '----"run_already" can be set "True" default is "False" \n'
                  '----"check_overlay" can be set "True" default is "False" \n'
                  '----"plot_RMSE" can be set "True" default is "False" \n'
                   '----"check_sequence_order" can be set "True" default is "False" \n'
                    '----camera matrix for initial guess can be set with cx cy fx fy \n'
                  'calibrateCameras.py -i <my_calibration_bags> <run_already=True> <check_overlay=False> <plot_only_RMSE=True> <check_sequence_order=True>\n'
                  'cx=<int> cy=<int> fx=<int> fy=<int>\n')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            bagfile = arg
    path_and_name = str(bagfile)
    if path_and_name[0] != '/':
        path_and_name = '/' + path_and_name
    run_already = False
    check_overlay = False
    plot_RMSE = False
    check_sequence_order = False
    cx, cy, fx, fy = 340, 240, 570, 570
    if len(args) != 0:
        print('args....................................', args)
        for arg in args:
            if arg[:3] == 'run':
                run_already = True
                print('arg..................', run_already)
            if arg[:7] == 'check_o':
                check_overlay = True
                print('arg..................', check_overlay)
            if arg[:3] == 'plo':
                plot_RMSE = True
                print('arg..................', plot_RMSE)
            if arg[:7] == 'check_s':
                check_sequence_order = True
                print('arg..................', check_sequence_order)
            if arg[:2] == 'cx':
                cx = int(arg[3:])
                print('arg..................', cx)
            if arg[:2] == 'cy':
                cy = int(arg[3:])
                print('arg..................', cy)
            if arg[:2] == 'fx':
                fx = int(arg[3:])
                print('arg..................', fx)
            if arg[:2] == 'fy':
                fy = int(arg[3:])
                print('arg..................', fy)

    print('run_already', run_already, '\ncheck_overlay', check_overlay,
          '\nplot_only_RMSE', plot_RMSE, '\ncheck_sequence_order', check_sequence_order)

    l515_3D_pattern_coordinates = mf.load_pattern_coordinates_from_di(path_and_name)
    if run_already:
        a, optris_2D_pattern_coordinates, b = mf.load_pattern_coordinates_from_ir(path_and_name, True)
    else:
        a, optris_2D_pattern_coordinates, b = mf.extract_pattern_coordinates_from_ir(path_and_name, True, True)

    l1 = len(optris_2D_pattern_coordinates)
    l2 = len(optris_2D_pattern_coordinates[0])
    optris_2D_data = np.asarray(optris_2D_pattern_coordinates).reshape(l1*l2, 2).T
    initial_guess = [0, 0, 0, 0, 0, 0, 1, 1, 1, cx, cy, fx, fy]
    result = scipy.optimize.minimize(mf.error_function_extrinsic_calibration, initial_guess,
                                     args=(l515_3D_pattern_coordinates, optris_2D_data),
                                    method='SLSQP', jac=None, bounds=None, tol=None, callback=None,
                                    options={'maxiter': 20000, 'ftol': 1e-08, 'iprint': 1, 'disp': False, 'eps': 1e-07,
                                                  'finite_diff_rel_step': None})

    if result.success:
        fitted_params = np.asarray(result.x)
        print(fitted_params)
        print('initial guess', initial_guess)
    else:
        raise ValueError(result.message)
    l515_3D_to_3D = mf.threeD2ThreeD(fitted_params[0], fitted_params[1], fitted_params[2],
                                     fitted_params[3], fitted_params[4], fitted_params[5],
                                     fitted_params[6], fitted_params[7], fitted_params[8], l515_3D_pattern_coordinates)
    l515_3D_to_2D = mf.threeD2TwoD(fitted_params[9], fitted_params[10], fitted_params[11], fitted_params[12], l515_3D_to_3D, True)


    T = mf.transformation_matrix(fitted_params[0], fitted_params[1], fitted_params[2],
                                fitted_params[3], fitted_params[4], fitted_params[5],
                                fitted_params[6], fitted_params[7], fitted_params[8])
    mtx_opt_640x480 = np.asarray([[fitted_params[11],                 0, fitted_params[9]],
                                  [                0, fitted_params[12], fitted_params[10]],
                                  [                0,                 0,                1]])
    print('extrinsic transformation matrix, 3D camera frame into 2D camera frame\n', T)
    print('intrisic camera matrix, 2D camera\n', mtx_opt_640x480)
    print('intrisic camera matrix, 3D camera\n', mf.get_di_intrinsics(path_and_name))
    rmse_list = mf.plot_overlayed_point_sets(path_and_name, optris_2D_data, l515_3D_to_2D, check_overlay,
                                             plot_RMSE, check_sequence_order)

if __name__ == "__main__":
    main(sys.argv[1:])