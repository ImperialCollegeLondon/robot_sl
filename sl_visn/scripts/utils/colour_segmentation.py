import cv2
import numpy as np

class ColourSegmentation:
    """
    Morphology Operations: 0: Erosion; 1: Dilation; 2: Opening; 3: Closing
    """
    thresholds = np.array([[[0,0,0], [0,0,0]]])
    def __init__(self, thresh_l, thresh_h, image_grabber, kernel_size=3, morph_op=[2,3], live_adjust=True):
        self.thresh_l = thresh_l
        self.thresh_h = thresh_h
        self.update_thresholds(thresh_l, thresh_h)
        self.image_grabber = image_grabber
        self.kernel_size = kernel_size
        self.morphology_operation = morph_op
        if live_adjust:
            self.set_thresholds()

    def set_thresholds(self):
        cv2.namedWindow('Mask')
        cv2.createTrackbar('B LOW :', 'Mask', self.thresh_l[0], 255, self.nothing)
        cv2.createTrackbar('B HIGH:', 'Mask', self.thresh_h[0], 255, self.nothing)
        cv2.createTrackbar('G LOW :', 'Mask', self.thresh_l[1], 255, self.nothing)
        cv2.createTrackbar('G HIGH:', 'Mask', self.thresh_h[1], 255, self.nothing)
        cv2.createTrackbar('R LOW :', 'Mask', self.thresh_l[2], 255, self.nothing)
        cv2.createTrackbar('R HIGH:', 'Mask', self.thresh_h[2], 255, self.nothing)

        while True:
            img = self.image_grabber()
            
            # simple scheme to tune mask with trackbars
            self.thresh_l[0] = cv2.getTrackbarPos('B LOW :','Mask')
            self.thresh_h[0] = cv2.getTrackbarPos('B HIGH:','Mask')
            self.thresh_l[1] = cv2.getTrackbarPos('G LOW :','Mask')
            self.thresh_h[1] = cv2.getTrackbarPos('G HIGH:','Mask')
            self.thresh_l[2] = cv2.getTrackbarPos('R LOW :','Mask')
            self.thresh_h[2] = cv2.getTrackbarPos('R HIGH:','Mask')
            self.update_thresholds(self.thresh_l, self.thresh_h)
            mask = self.predict_img(img)
            # cv2.imshow("Mask", rescale(mask, 0.3))
            cv2.imshow("Mask", cv2.resize(mask, (350, int(mask.shape[0]/mask.shape[1]*350)), interpolation = cv2.INTER_AREA))
            cv2.imshow("Original", img)

            # Continue until the user presses ESC key
            key = cv2.waitKey(1) 
            if key == 27:
                cv2.destroyAllWindows()
                break

    @staticmethod
    def nothing(val):
        pass
    
    def update_thresholds(self, thresh_l, thresh_h):
        thresholds = np.array([[[0,0,0], [0,0,0]]])
        for id, (high,low) in enumerate(zip(thresh_h,thresh_l)):
            if high>low:
                for t in thresholds:
                    t[0, id] = low
                    t[1, id] = high
            else:
                ## duplicate the thresholds
                thresholds = np.repeat(thresholds, 2, axis=0)
                for j, t in enumerate(thresholds):
                    if j%2==0:
                        t[0, id] = low
                        t[1, id] = 255
                    else:
                        t[0, id] = 0
                        t[1, id] = high
        self.thresholds = thresholds

    """ generate image mask according to given thresholds """
    def predict_img(self, img):
        # print(thresholds)
        mask = np.zeros(img.shape[:2], dtype=np.uint8)
        for t in self.thresholds:
            mask = cv2.bitwise_or(mask, cv2.inRange(img, t[0], t[1]))

        # post processing to get a cleaner mask
        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (self.kernel_size,self.kernel_size))
        for op in self.morphology_operation:
            if op==0:
                mask = cv2.morphologyEx(mask, cv2.MORPH_ERODE, kernel)
            elif op==1:
                mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)
            elif op==2:
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel) # erosion followed by dilation
            elif op==3:
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel) # dilation followed by erosion
            else:
                print("Unknown Morphology Operation!")
        # save the mask
        # cv2.imwrite("/catkin_ws/mask.png", mask)
        return mask

    """ generate image mask according to given threshold """
    def colourSegmentation(self, img, lower, upper):
        # create a mask by colour
        return cv2.inRange(img, lower, upper)
