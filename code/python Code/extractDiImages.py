import myfunctions as mf
import sys, getopt




def main(argv):
    try:
        opts, args = getopt.getopt(argv, "hi:o:", "ifile=")
    except getopt.GetoptError:
        print('extractDiImages.py -i <my_calibration_bags>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('----no options----\n '
                  'extractDiImages.py -i <my_calibration_bags>')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            bagfile = arg
    path_and_name = str(bagfile)
    if path_and_name[0] != '/':
        path_and_name = '/' + path_and_name
    print('depth images are being extracted........')
    mf.extract_di_images_for_calibration(path_and_name)

if __name__ == "__main__":
    main(sys.argv[1:])