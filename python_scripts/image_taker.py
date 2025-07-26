import cv2

def main():
    cap = cv2.VideoCapture(0)
    nr_images = 0
    while True:
        if not cap.isOpened():
            print("Cannot open camera")
            exit()
        ret, frame = cap.read()
        cv2.imshow("Press \'r\' to capture image",frame)
        key = cv2.waitKey(1)
        if  key == ord('q'):
            break
        elif key == ord('r'):
            nr_images += 1
            print("Images taken: {}".format(nr_images))
            img_name = "calibration_img_{}.png".format(nr_images)
            cv2.imwrite("calibration_imgs/{}".format(img_name), frame)
    cap.release()
    cv2.destroyAllWindows()
            

if __name__ == "__main__":
    main()