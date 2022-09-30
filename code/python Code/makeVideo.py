import cv2
import glob
import sys, getopt

def main(argv):
    try:
        opts, args = getopt.getopt(argv, "hi:o:", "ifile=")
    except getopt.GetoptError:
        print('makeVideo.py -i <my_picture_sequence>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('----no options----\nmakeVideo.py -i <my_picture_sequence>')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            bagfile = arg
    path_and_name = str(bagfile)
    if path_and_name[0] != '/':
        path_and_name = '/' + path_and_name
    if path_and_name[-1] != '/':
        path_and_name = path_and_name + '/'
    print('video is being rendered........')
    img_array = []
    filenames = sorted(glob.glob(path_and_name + '*.*'))
    for filename in filenames:
        print(filename)
        img = cv2.imread(filename)
        height, width, layers = img.shape
        size = (width, height)
        img_array.append(img)
    out = cv2.VideoWriter(path_and_name + 'out.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 15, size)
    for i in range(len(img_array)):
        out.write(img_array[i])
    out.release()

if __name__ == "__main__":
    main(sys.argv[1:])

