import myfunctions as mf
import sys, getopt


def main(argv):

    try:
        opts, args = getopt.getopt(argv, "hi:o:", "ifile=")
    except getopt.GetoptError:
        print('extractIrPattern.py -i <my_calibration_bags>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('\n----"check_pattern" can be set "True" or "False" \n'
                  '----"display_time", unit is [ms]\n'
                '----sequence and number of arguments is arbitrary" \n'
                  'extractIrPattern.py -i <my_calibration_bags> <show_patterns=True/False> <display_time=[ms]> \n')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            bagfile = arg
    path_and_name = str(bagfile)
    if path_and_name[0] != '/':
        path_and_name = '/' + path_and_name
    print('Bagfile file used: "' + path_and_name + '"')
    show = False
    time = 100
    if len(args) != 0:
        for arg in args:
            if arg == 'show_patterns=True':
                show = True
                print(show)
            if arg[:4] == 'disp':
                time = int(arg[13:])
                print(time)
    mf.extract_pattern_coordinates_from_ir(path_and_name, write_coordinates_to_disk=True, extract_from_calibrated_images=False, show_patterns=show, display_time=time)

if __name__ == "__main__":
    main(sys.argv[1:])