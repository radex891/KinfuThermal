import myfunctions as mf
import sys, getopt


def main(argv):

    try:
        opts, args = getopt.getopt(argv, "hi:o:", "ifile=")
    except getopt.GetoptError:
        print('extractIrImages.py -i <my_calibration_bags>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('----check quality of still image phase----\n '
                  'extractIrImages.py -i <my_calibration_bags> check_for_still_images=True')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            bagfile = arg
    path_and_name = str(bagfile)
    if path_and_name[0] != '/':
        path_and_name = '/' + path_and_name
    print('Bagfile file used: "' + path_and_name + '"')
    still = False
    if len(args) != 0:
        if args[0] == 'check_for_still_images=True':
            still = True
    mf.extract_ir_images_for_calibration(path_and_name, minimum_still_images=20, check_for_still_images=still)

if __name__ == "__main__":
    main(sys.argv[1:])