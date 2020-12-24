#!/usr/bin/env python
# coding=utf8

import numpy as np
import cv2
import os.path
import time

class ImageEstimation():
    def __init__(self, _MIN_MATCH_COUNT = 40, _threshold = 500, _use_image = False, _size_image = 0., _image_path = "", show_image = False):

        # init params
        self.MIN_MATCH_COUNT = _MIN_MATCH_COUNT
        self.threshold = _threshold
        self.use_image = _use_image

        # initialization
        self.input_mode = False
        self.track_mode = False
        self.show_image = show_image

        self.box_pts = []

        self.frame = None
        self.img_object = None

        self.matches = []

        self.box_pts = []
        self.size_image = _size_image

        self.max_dist = 10.0
        self.camera_parameters = np.array(
            [[653.564007, 0.000000, 326.174970], [0.000000, 655.878899, 247.761618], [0, 0, 1]])
        self.camera_distortion_param = np.array([-0.426801, 0.249082, -0.002705, -0.001600, 0.000000])

        # Initiate SIFT detector
        self.sift = cv2.xfeatures2d.SIFT_create()

        if self.use_image:
            print("get image: ", _image_path)
            self.img_object = cv2.imread(_image_path,0)          # queryImage

            self.track_mode = True
            self.kp1, self.des1 = self.sift.detectAndCompute(self.img_object, None)
            # if self.show_image:
            #     cv2.namedWindow('target image', cv2.WINDOW_NORMAL)
            #     cv2.imshow("target image", self.img_object)

    def update(self, frame):
        self.frame = frame

        blur_state, val_blur = self.is_blur(frame)
        if blur_state:
            print("image blur: %d" %val_blur)
            return frame, None, None

        translation = None
        rotation_rad = None
        # press i to enter input mode
        # if k == ord('i') and self.use_image is False:
        #     print("key i")
        #     self.track_mode = False
        #     # select object by clicking 4-corner-points
        #     del self.box_pts[:]
        #     self.select_object_mode()
        #
        #     # set the boundary of reference object
        #     pts2, right_bound, left_bound, lower_bound, upper_bound = self.set_boundary_of_reference(self.box_pts)
        #
        #     # do perspective transform to reference object
        #     self.img_object = self.input_perspective_transform(self.box_pts, pts2, right_bound, left_bound, lower_bound,
        #                                                        upper_bound)
        #
        #     if self.show_image:
        #         cv2.imshow("show", self.img_object)
        #     self.track_mode = True
        #     self.kp1, self.des1 = self.sift.detectAndCompute(self.img_object, None)

        # if k == ord('s'):
        #     print("save image")
        #     cv2.imwrite(os.path.dirname(__file__)+'/save_image.jpg', self.img_object)
        #
        # # track mode is run immediately after user selects 4-corner-points of object
        if self.track_mode is True:
            # feature detection and description

            kp2, des2 = self.sift.detectAndCompute(frame, None)
            # feature matching
            self.matches = self.sift_force_feature_matcher(self.kp1, self.des1, kp2, des2)
            print("Find matches: %d/%d" % (len(self.matches), self.MIN_MATCH_COUNT))
            if len(self.matches) > self.MIN_MATCH_COUNT:
                # find homography matrix
                M, mask = self.find_homography_object(self.kp1, kp2, self.matches)

                # apply homography matrix using perspective transformation
                try:
                    corner_camera_coord, center_camera_coord, object_points_3d, center_pts = self.output_perspective_transform(self.img_object,self.size_image, M)

                    if self.filter_corner(corner_camera_coord) is False:
                        return frame, None, None

                    # solve pnp using iterative LMA algorithm

                    rotation_rad, translation = self.iterative_solve_pnp(object_points_3d, corner_camera_coord,
                                                                             self.camera_parameters,
                                                                             self.camera_distortion_param)
                    dist = np.linalg.norm(translation)
                    if dist > self.max_dist:
                        print("dist:", dist)
                        return frame, None, None

                    # convert to degree
                    rotation = rotation_rad * 57.2957795131
                    # if self.show_image:
                    # draw box around object
                    self.frame = self.draw_box_around_object(self.frame, corner_camera_coord)

                    # show object position and orientation value to frame
                    self.frame = self.put_position_orientation_value_to_frame(self.frame, translation, rotation)
                except:
                    return frame, None, None

        return self.frame, translation, rotation_rad

    # callback function for selecting object by clicking 4-corner-points of the object
    def select_object(self, event, x, y, flags, param):
        if self.input_mode and event == cv2.EVENT_LBUTTONDOWN and len(self.box_pts) < 4:
            self.box_pts.append([x, y])
            self.frame = cv2.circle(self.frame, (x, y), 4, (0, 255, 0), 2)


    # selecting object by clicking 4-corner-points
    def select_object_mode(self):
        self.input_mode = True

        # frame_static = self.frame.copy()

        # while len(self.box_pts) < 4:
        #     cv2.imshow("frame", self.frame)
        #     cv2.waitKey(1)

        self.input_mode = False


    # setting the boundary of reference object
    def set_boundary_of_reference(self, box_pts):
        ### upper bound ###
        if box_pts[0][1] < box_pts[1][1]:
            upper_bound = box_pts[0][1]
        else:
            upper_bound = box_pts[1][1]

        ### lower bound ###
        if box_pts[2][1] > box_pts[3][1]:
            lower_bound = box_pts[2][1]
        else:
            lower_bound = box_pts[3][1]

        ### left bound ###
        if box_pts[0][0] < box_pts[2][0]:
            left_bound = box_pts[0][0]
        else:
            left_bound = box_pts[2][0]

        ### right bound ###
        if box_pts[1][0] > box_pts[3][0]:
            right_bound = box_pts[1][0]
        else:
            right_bound = box_pts[3][0]

        upper_left_point = [0, 0]
        upper_right_point = [(right_bound - left_bound), 0]
        lower_left_point = [0, (lower_bound - upper_bound)]
        lower_right_point = [(right_bound - left_bound), (lower_bound - upper_bound)]

        pts2 = np.float32([upper_left_point, upper_right_point, lower_left_point, lower_right_point])

        # display dimension of reference object image to terminal
        print (pts2)

        return pts2, right_bound, left_bound, lower_bound, upper_bound

    # doing perspective transform to reference object
    def input_perspective_transform(self, box_pts, pts2, right_bound, left_bound, lower_bound, upper_bound):
        pts1 = np.float32(box_pts)
        M = cv2.getPerspectiveTransform(pts1, pts2)
        img_object = cv2.warpPerspective(self.frame, M, ((right_bound - left_bound), (lower_bound - upper_bound)))
        return cv2.cvtColor(img_object, cv2.COLOR_BGR2GRAY)


    # feature matching using Brute Force
    def sift_force_feature_matcher(self, kp1, des1, kp2, des2):
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = []
        try:
            matche_raw = flann.knnMatch(des1, des2, k=2)
            # store all the good matches as per Lowe's ratio test.
        except:
            return matches
        for m, n in matche_raw:
            if m.distance < 0.75 * n.distance:
                matches.append(m)
        return matches

    # finding homography matrix between reference and image frame
    def find_homography_object(self, kp1, kp2, matches):
        src_pts = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        return M, mask


    # applying homography matrix as inference of perpective transformation
    def output_perspective_transform(self, img_object, marker_lenght, M):
        h, w = img_object.shape
        corner_pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
        center_pts = np.float32([[w / 2, h / 2]]).reshape(-1, 1, 2)

        x0 = marker_lenght / 2
        y0 = marker_lenght / 2

        x1 = -marker_lenght / 2
        y1 = -marker_lenght / 2
        corner_pts_3d = np.float32([[x1, y0, 0], [x0, y0, 0], [x0, y1, 0], [x1, y1, 0]])

        corner_camera_coord = cv2.perspectiveTransform(corner_pts, M)  ###
        center_camera_coord = cv2.perspectiveTransform(center_pts, M)

        return corner_camera_coord, center_camera_coord, corner_pts_3d, center_pts


    # solving pnp using iterative LMA algorithm
    def iterative_solve_pnp(self, object_points, image_points, camera_parameters, camera_distortion_param):
        # image_points = image_points.reshape(-1, 2)
        retval, rotation, translation = cv2.solvePnP(object_points, image_points, camera_parameters,
                                                     camera_distortion_param,flags=cv2.SOLVEPNP_IPPE_SQUARE)

        return rotation, translation

    # drawing box around object
    def draw_box_around_object(self, frame, dst):
        for i in range(len(dst)):
                frame = cv2.putText(frame, (i+1).__str__(), (dst[i][0][0], dst[i][0][1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

        return cv2.polylines(frame, [np.int32(dst)], True, 255, 3)



    def is_blur(self, image):
        """
        Retund image state
        :param image:
        :param threshold:
        :return:
        """
        val = cv2.Laplacian(image, cv2.CV_64F).var()
        if val < self.threshold:
            return True, val
        else:
            return False, val

    def filter_corner(self, points):
        "Checking the quality of found points"

        if len(points) < 4:
            return False
        x1 = points[0][0][0]
        y1 = points[0][0][1]

        x2 = points[1][0][0]
        y2 = points[1][0][1]

        x3 = points[2][0][0]
        y3 = points[2][0][1]

        x4 = points[3][0][0]
        y4 = points[3][0][1]

        dist_1 = np.linalg.norm([x2-x1,y2-y1])
        dist_2 = np.linalg.norm([x3-x2,y3-y2])
        dist_3 = np.linalg.norm([x4-x3,y4-y3])
        dist_4 = np.linalg.norm([x1-x4,y1-y4])

        dev_1 = dist_2 / dist_1
        dev_2 = dist_4 / dist_3
        dev_3 = dist_4 / dist_2
        dev_4 = dist_3 / dist_2
        min_v = 0.6
        max_v = 2.0

        if min_v > dev_1 or dev_1 > max_v or \
                min_v > dev_2 or dev_2 > max_v or \
                min_v > dev_3 or dev_3 > max_v or \
                min_v > dev_4 or dev_4 > max_v:
            print "filter error"
            print dev_1, dev_2, dev_3, dev_4
            return False
        return True

    def put_position_orientation_value_to_frame(self, frame, translation, rotation):
        # showing object position and orientation value to frame
        font = cv2.FONT_HERSHEY_SIMPLEX

        cv2.putText(frame, 'position(cm)', (10, 30), font, 0.7, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, 'x:' + str(round(translation[0], 2)), (250, 30), font, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, 'y:' + str(round(translation[1], 2)), (350, 30), font, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, 'z:' + str(round(translation[2], 2)), (450, 30), font, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

        cv2.putText(frame, 'orientation(degree)', (10, 60), font, 0.7, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, 'x:' + str(round(rotation[0], 2)), (250, 60), font, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, 'y:' + str(round(rotation[1], 2)), (350, 60), font, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, 'z:' + str(round(rotation[2], 2)), (450, 60), font, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, 'Find matches: ', (10, 90), font, 0.7, (0, 255, 0), 1, cv2.LINE_AA)

        cv2.putText(frame, "%s/%s" % (len(self.matches), self.MIN_MATCH_COUNT), (250, 90), font, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

        return frame