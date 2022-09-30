import myfunctions as mf
import sys, getopt

def main(argv):
    try:
        opts, args = getopt.getopt(argv, "hi:o:", "ifile=")
    except getopt.GetoptError:
        print('extractDiPattern.py -i <my_calibration_bags>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('\n----filter out background, unit is [mm]----\n'
                  '----"check_pattern" can be set "True" or "False" \n'
                  '----"visualize_stepbystep" can be set "True" or "False" \n'
                  '----sequence and number of arguments is arbitrary" \n'
                  'extractDiPattern.py -i <my_calibration_bags> <background_filter_value=[mm]> <check_pattern=True/False> <visualize_stepbystep=True/False>\n')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            bagfile = arg
    path_and_name = str(bagfile)
    if path_and_name[0] != '/':
        path_and_name = '/' + path_and_name
    background_filter = 2200
    check_pattern = False
    visualize_stepbystep = False
    if len(args) != 0:
        for arg in args:
            if arg[:4] == 'back':
                background_filter = int(arg[24:])
                print('arg..................', background_filter)
            if arg == 'check_pattern=True':
                check_pattern  = True
                print('arg..................', check_pattern)
            if arg == 'visualize_stepbystep=True':
                visualize_stepbystep = True
                print('arg..................', visualize_stepbystep)
    mf.extract_pattern_coordinates_from_di(path_and_name, background_filter, check_pattern, visualize_stepbystep)
if __name__ == "__main__":
    main(sys.argv[1:])