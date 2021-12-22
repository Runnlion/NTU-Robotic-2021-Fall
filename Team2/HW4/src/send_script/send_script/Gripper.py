import numpy as np
import cv2 as cv
import ImageProcessing


class ArmControl:
    def __init__(self, send_script, set_io) -> None:
        self.send_script = send_script
        self.set_io = set_io

    def release(self):
        self.set_io(0.0)

    def grab(self):
        self.set_io(1.0)

    def move_to_pose(self, pose):
        """Move to a specific pose
        @Params:
            a_pos: 1x4 matrix in homogenous form, describing position
            a_orien: 1x3 matrix, describing Euler angles of the orientation 
                of the end-effector
        """
        if pose[2] < 105:
            pose[2] = 105
            print("Height too low: ", pose[2])
        targetP1 = '{0}, {1}, {2}, {3}, {4}, {5}'.format(
            pose[0], pose[1], pose[2], pose[3], pose[4], pose[5])
        script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
        self.send_script(script1)


class Gripper:
    def __init__(self, arm) -> None:
        """Calibration process (or load previous calibrated file), including
        1. intrinsic calibration if not done yet.
        2. extrinsic calibrarion (control by the flag)
        """

        self.arm = arm

        self.mbTc = self.img_loader.mbTc

    def __grab_and_release(self, b_obj, b_psi_obj, b_target, b_psi_tar, zoffset_after_release):
        orientation = [-180, 0, 135]

        # TODO: the z-angle need to be determined
        b_grab_ori = np.array([orientation[0], orientation[1], b_psi_obj])
        b_grab_pose = np.hstack([b_obj.T[0, :3], b_grab_ori])

        b_ready_pose = b_grab_pose.copy()
        b_ready_pose[2] += zoffset_after_release
        # Move to target
        self.arm.move_to_pose(b_ready_pose)
        self.arm.release()
        # Move downward
        self.arm.move_to_pose(b_grab_pose)
        self.arm.grab()

        b_release_ori = np.array([orientation[0], orientation[1], b_psi_tar])
        b_release_pose = np.hstack([b_target.T[0, :3], b_release_ori])

        b_ready_pose = b_release_pose.copy()
        b_ready_pose[2] += zoffset_after_release
        # Go to target position
        self.arm.move_to_pose(b_ready_pose)
        self.arm.release()
        # Move down
        self.arm.move_to_pose(b_release_pose)

    def __transform_img_frame_to_arm_frame(self, im_pt):
        depth = None
        c_pt = self.img_loader.get_c_pt_im(im_pt, depth)
        b_pt = self.mbTc @ c_pt
        return b_pt

    def run(self, img):
        # Find the largest object and set it as base
        mask = np.zeros(img.shape[:2], dtype=np.uint8)
        im_pt, im_pa, cnt = ImageProcessing.find_largest_object(
            img=img, mask=None)
        mask = cv.drawContours(mask, [cnt], -1, 255)

        # Transform to base frame
        b_target = self.__transform_img_frame_to_arm_frame(im_pt)

        height_offset = np.array([[0, 0, 15]]).T
        # Iteration until no object can be found
        for i in range(10):
            im_pt, im_pa, cnt = ImageProcessing.find_largest_object(
                img=img, mask=mask)
            if cnt is None:
                break

            # Update the mask
            mask = cv.drawContours(mask, [cnt], -1, 255)

            b_obj = self.__transform_img_frame_to_arm_frame(im_pt)

            # The tower grows
            b_target += height_offset * i
            self.__grab_and_release(
                b_obj, obj_orien, b_target, target_orien, zoffset_after_release=50)

        cv.destroyAllWindows()


def main():
    gripper = Gripper()
    gripper.run()


if __name__ == "__main__":
    main()
