import zbar

from PIL import Image
import cv2

def main():
    # Select and start Camera.
    DEVICE_NO = 0
    capture = cv2.VideoCapture(DEVICE_NO)
    if not capture:
        print "Failed to open camera number %d" %(DEVICE_NO)
        return False
    while True:
        if cv2.waitKey(100) & 0xFF == ord('q'):       #q to quit
            cv2.destroyAllWindows()
            break

        ret, frame = capture.read()
        if ret==False:
            print "Error !"
            return False
        cv2.imshow('Video Capture', frame)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        image = Image.fromarray(gray)
        width, height = image.size
        zbar_image = zbar.Image(width, height, 'Y800', image.tobytes())

        scanner = zbar.ImageScanner()
        scanner.scan(zbar_image)

        for decoded in zbar_image:
            print(decoded.data)
    return True

if __name__ == "__main__":
    print main()
