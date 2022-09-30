import numpy as np
import sys, getopt
import myfunctions as mf
import command
import os

# source /home/k/PycharmProjects/pythonProject/venv/bin/activate
# path_and_name = '/home/k/Dokumente/l515bmw/1.bag'
# path_and_name = '/home/k/Dokumente/test_main_args/1.bag'
# path_and_name = '/home/k/Dokumente/buerotimobmw/bmw_sommer2.bag'

def main(argv):
    topic1 = '/optris/image_default'
    topic2 = '/camera/depth/image_rect_raw'
    topic4 = '/optris/camera_info'
    topic5 = '/camera/depth/camera_info'
    topic6 = '/optris/flag_state'
    topics_in_bag = topic1, topic2, topic4, topic5, topic6
    topics_str = 'topic1=', 'topic2=', 'topic4=', 'topic5=', 'topic6='
    bagfile = ''
    try:
        opts, args = getopt.getopt(argv, "hi:o:", "ifile=")
    except getopt.GetoptError:
        print('preprocessRosbag.py -i <bagfile>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('----change one topic or multiple topics and/or drift_factor----\n '
                  '"preprocessRosbag.py -i <bagfile> topic<no.>=<topic> drift_factor=<float>"')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            bagfile = arg
    path_and_name = str(bagfile)
    if path_and_name[0] != '/':
        path_and_name = '/' + path_and_name
    print('Bagfile file used: "' + path_and_name + '"')
    if len(args) == 0:
        for i in range(len(topics_in_bag)):
            print(topics_str[i] + topics_in_bag[i])
    else:
        for arg in args:
            if arg[5] == '1':
                topic1 = (arg.rsplit('=', 1)[-1])
            elif arg[5] == '2':
                topic2 = (arg.rsplit('=', 1)[-1])
            elif arg[5] == '4':
                topic4 = (arg.rsplit('=', 1)[-1])
            elif arg[5] == '5':
                topic5 = (arg.rsplit('=', 1)[-1])
            elif arg[5] == '6':
                topic6 = (arg.rsplit('=', 1)[-1])
            elif arg[:12] == 'drift_factor':
                drift_factor = float(arg.rsplit('=', 1)[-1])
                print('new drift factor: ', drift_factor)

        topics_in_bag = topic1, topic2, topic4, topic5, topic6
        for i in range(len(topics_in_bag)):
            print(topics_str[i] + topics_in_bag[i])

    camera_matrix_optris = np.loadtxt(path_and_name.rsplit('/', 1)[0] + '/camera_matrix_optris.csv', delimiter=",")
    distortion_params_optris = np.loadtxt(path_and_name.rsplit('/', 1)[0] + '/distortion_params_optris.csv', delimiter=",")
    transformation_vector_realsense = np.loadtxt(path_and_name.rsplit('/', 1)[0] + '/transformation_vector_realsense.csv', delimiter=",")
    camera_matrix_optris_updated = np.loadtxt(path_and_name.rsplit('/', 1)[0] + '/camera_matrix_optris_updated.csv', delimiter=",")
    print('camera_matrix_optris\n', camera_matrix_optris)
    print('distortion_params_optris\n', distortion_params_optris)
    print('transformation_vector_realsense\n', transformation_vector_realsense)
    print('camera_matrix_optris_updated\n', camera_matrix_optris_updated)

    #mf.write_cameraInfo_to_csv(path_and_name, topic5, topic4, transformation_vector_realsense, camera_matrix_optris_updated)
    print("camera infos written")
    #mf.write_images_from_rosbag_to_folder(path_and_name, camera_matrix_optris, distortion_params_optris, topics_in_bag)
    print("images extracted from rosbag")


    mf.assign_and_rename_images(path_and_name, drift_factor=0.996, undo_previous_rename=False)
    print("timestamps assigned")
    #os.system("roslaunch turtle_tf2 turtle_tf2_demo.launch")

if __name__ == "__main__":
    main(sys.argv[1:])

