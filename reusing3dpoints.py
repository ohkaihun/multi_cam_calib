from typing import *
import csv
# from utm import utmconv
import cv2
import numpy as np
from glob import glob
import os
import g2o

# def extract_gps(file):
#     coords = []
#     with open(file, 'r') as f:
#         log = csv.reader(f)
#         header = iter(log).__next__()
#         idx_rec = header.index("CUSTOM.isVideo")
#         idx_lat = header.index("OSD.latitude")
#         idx_lon = header.index("OSD.longitude")
#         idx_height = header.index("OSD.altitude [m]")
#         for r in log:
#             if r[idx_rec] == "Recording":
#                 coords.append((float(r[idx_lat]), float(r[idx_lon]), float(r[idx_height])))
#             elif r[idx_rec] == "Stop":
#                 return coords
#
#
# def geodetic_to_utm(coords):
#     coords_utm = []
#     conv = utmconv()
#     for lat, lon, alt in coords:
#         coords_utm.append((*conv.geodetic_to_utm(lat, lon)[-2:], alt))
#     return coords_utm
#

def quarternion_to_rotation_matrix(q):
    """
    The formula for converting from a quarternion to a rotation
    matrix is taken from here:
    https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
    """
    qw = q.w()
    qx = q.x()
    qy = q.y()
    qz = q.z()
    R11 = 1 - 2 * qy ** 2 - 2 * qz ** 2
    R12 = 2 * qx * qy - 2 * qz * qw
    R13 = 2 * qx * qz + 2 * qy * qw
    R21 = 2 * qx * qy + 2 * qz * qw
    R22 = 1 - 2 * qx ** 2 - 2 * qz ** 2
    R23 = 2 * qy * qz - 2 * qx * qw
    R31 = 2 * qx * qz - 2 * qy * qw
    R32 = 2 * qy * qz + 2 * qx * qw
    R33 = 1 - 2 * qx ** 2 - 2 * qy ** 2
    R = np.array([[R11, R12, R13], [R21, R22, R23], [R31, R32, R33]])
    return R
#

class FrameIterator:
    def __init__(self, filename: str):
        self.cap = cv2.VideoCapture(filename)

    def frame_generator(self) -> np.ndarray:
        i = 0
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            if i % 50 == 0:
                yield frame
            i += 1
        self.cap.release()

    def main(self) -> None:
        # For every frame
        for t, frame in enumerate(self.frame_generator()):
            print(t)
            cv2.imwrite(f"output/fifties/f{t:2d}.jpg", frame)


class Point:
    def __init__(self, point_id: int, point: np.array):
        self.id = point_id
        self.point = point

    @property
    def x(self) -> float:
        return self.point[0]

    @property
    def y(self) -> float:
        return self.point[1]

    @property
    def z(self) -> float:
        return self.point[2]


class Observation:
    def __init__(self, point_id: int, camera_id: int, point: np.array):
        self.point_id = point_id
        self.camera_id = camera_id
        self.point = point

    @property
    def u(self) -> float:
        return self.point[0]

    @property
    def v(self) -> float:
        return self.point[1]


class Camera:
    def __init__(self, camera_id: int, R: np.array = np.eye(3), t: np.array = np.zeros(3), fixed: bool = False):
        self.id = camera_id
        self.R = R
        self.t = t
        self.fixed = fixed

    @property
    def pose(self) -> np.array:
        pose = np.eye(4)
        pose[:3, :3] = self.R
        pose[:3, 3] = self.t.ravel()
        return pose

    @pose.setter
    def pose(self, pose: np.array):
        self.R = pose[:3, :3]
        self.t = pose[:3, 3]


class Map:
    def __init__(self, K: np.array):
        self.points: List[Point] = []
        self.observations: List[Observation] = []
        self.cameras: List[Camera] = []
        self.K = K

    def remove_camera(self, cam_id: int):
        before = len(self.observations)
        self.cameras = [cam for cam in self.cameras if cam.id != cam_id]
        self.observations = [obs for obs in self.observations if obs.camera_id != cam_id]
        return before - len(self.observations)

    def reproj_err(self) -> float:
        id2p = {p.id: p for p in self.points}
        id2c = {c.id: c for c in self.cameras}

        sum = 0
        for obs in self.observations:
            cam = id2c[obs.camera_id]
            if cam.id == 0:
                continue

            p = id2p[obs.point_id].point
            p_cam = cam.pose @ p
            t = self.K @ p_cam[:3]
            t /= t[2]
            dx = t[0] - obs.u
            dy = t[1] - obs.v
            err = dx ** 2 + dy ** 2

            sum += err
        return sum

    def bundle_adjustment(self):
        optimizer = g2o.SparseOptimizer()
        solver = g2o.BlockSolverSE3(g2o.LinearSolverEigenSE3())#LinearSolverCholmodSE3
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        optimizer.set_algorithm(solver)

        # print(self.K)
        focal_length = self.K[0, 0]
        principal_point = (self.K[0, 2], self.K[1, 2])
        baseline = 0
        cam = g2o.CameraParameters(focal_length, principal_point, baseline)
        cam.set_id(0)
        optimizer.add_parameter(cam)

        camera_vertices = {}

        for idx, camera in enumerate(self.cameras):
            # Use the estimated pose of the second camera based on the
            # essential matrix.
            pose = g2o.SE3Quat(camera.R, camera.t)

            # Set the poses that should be optimized.
            # Define their initial value to be the true pose
            # keep in mind that there is added noise to the observations afterwards.
            v_se3 = g2o.VertexSE3Expmap()
            v_se3.set_id(camera.id)
            v_se3.set_estimate(pose)
            #isfixed = False
            #if idx == 0:
            #    isfixed = True
            print("fixed?", camera.fixed)
            v_se3.set_fixed(camera.fixed)
            optimizer.add_vertex(v_se3)
            camera_vertices[camera.id] = v_se3
            # print("camera id: %d" % camera.camera_id)
        print("num cam_vertices:", len(camera_vertices))

        point_vertices = {}
        for point in self.points:
            # Add 3d location of point to the graph
            vp = g2o.VertexPointXYZ()
            vp.set_id(point.id)
            vp.set_marginalized(True)
            # Use positions of 3D points from the triangulation

            # point_temp = np.array(point.point, dtype=np.float64) #TODO: forventer 3x1, er 4x1 (scale)
            point_temp = np.array(point.point[0:3], dtype=np.float64)
            vp.set_estimate(point_temp)
            optimizer.add_vertex(vp)
            point_vertices[point.id] = vp
        print("num point_vertices:", len(point_vertices))

        print("# observations:", len(self.observations))
        for observation in self.observations: # Ikke sikker på at det her er rette syntax
            # Add edge from first camera to the point
            edge = g2o.EdgeProjectXYZ2UV()

            # 3D point
            edge.set_vertex(0, point_vertices[observation.point_id])
            # Pose of first camera
            edge.set_vertex(1, camera_vertices[observation.camera_id])

            edge.set_measurement(observation.point)
            edge.set_information(np.identity(2))
            edge.set_robust_kernel(g2o.RobustKernelHuber())

            edge.set_parameter_id(0, 0)
            optimizer.add_edge(edge)

        print('num vertices:', len(optimizer.vertices()))
        print('num edges:', len(optimizer.edges()))

        print('Performing full BA:')
        optimizer.initialize_optimization()
        optimizer.set_verbose(True)
        optimizer.optimize(40)
        optimizer.save("test.g2o");

        for idx, camera in enumerate(self.cameras):
            print("Camera ID:", self.cameras[idx].id)
            t = camera_vertices[camera.id].estimate().translation()
            print("Camera Translation Before BA:\n", self.cameras[idx].t)
            self.cameras[idx].t = t
            print("Camera Translation After BA:\n", self.cameras[idx].t)
            q = camera_vertices[camera.id].estimate().rotation()
            print("Camera Rotation Before BA:\n", self.cameras[idx].R)
            self.cameras[idx].R = quarternion_to_rotation_matrix(q)
            print("Camera Rotation After BA:\n", self.cameras[idx].R)

        for idx, point in enumerate(self.points):
            p = point_vertices[point.id].estimate()
            # It is important to copy the point estimates.
            # Otherwise I end up with some memory issues.
            # self.points[idx].point = p
            #print("point before:", self.points[idx].point)
            self.points[idx].point = np.copy(p)
            self.points[idx].point = np.hstack((self.points[idx].point, 1)) #fixes things
            #print("point after:", self.points[idx].point)


class Pipeline:
    def __init__(self):
        self.fd = cv2.SIFT_create()
        self.matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)

    @staticmethod
    def frame_iterator(dirname, scale):
        paths = sorted(glob(os.path.join(dirname, "*.jpg")))
        for p in paths:
            print("Image path:", p)
            img = cv2.imread(p)
            img = cv2.resize(img, (int(img.shape[1] * scale), int(img.shape[0] * scale)))
            yield img

    def use_points(self, p1, p2):
        self.p1 = p1
        self.p2 = p2

    def find_features(self, img):
        return self.fd.detectAndCompute(img, None)

    def match_features(self, des1, des2):
        return self.matcher.match(des1, des2)

    @staticmethod
    def find_E(p1, p2, K):
        return cv2.findEssentialMat(
            p1, p2,
            K,
            cv2.FM_RANSAC,
            0.99,
            1
        )

    @staticmethod
    def filter_by_matches(p1, p2, matches):
        p1_o = np.float32([p1[m.queryIdx].pt for m in matches])
        p2_o = np.float32([p2[m.trainIdx].pt for m in matches])
        return p1_o, p2_o

    @staticmethod
    def get_pose(E, p1, p2, K):
        _, R, t, _ = cv2.recoverPose(E, p1, p2, K)
        return R, t

    def triangulate(self, p1, p2, K, pose1, pose2):
        proj_mat1 = K @ pose1[:3]
        proj_mat2 = K @ pose2[:3]

        p_3d = cv2.triangulatePoints(proj_mat1, proj_mat2, p1.T, p2.T)
        p_3d /= p_3d[3]
        return p_3d

    @staticmethod
    def find_common_matches(prev_matches, matches):
        prev_idxs = set(m.trainIdx for m in prev_matches)
        common_matches = []
        new_matches = []
        for m in matches:
            if m.queryIdx in prev_idxs:
                common_matches.append(m)
            else:
                new_matches.append(m)
        return common_matches, new_matches

    def run(self, dirname):
        scale = 0.3
        uniqid = int(1e6)
        p_idx = uniqid
        for idx, img in enumerate(self.frame_iterator(dirname, scale)):
            #print("Iteration: ", idx)
            if idx == 0: # Initialize map
                f = 2676.1051390718389 * scale
                cx = -35.243952918157035 * scale + img.shape[1] / 2
                cy = -279.58562078697361 * scale + img.shape[0] / 2
                K = np.array([
                    [f, 0, cx],
                    [0, f, cy],
                    [0, 0, 1]
                ])
                prev_kp, prev_des = self.find_features(img)

                map = Map(K)
                map.cameras.append(Camera(0, fixed=True))

                continue

            # Find features and match with previous image
            kp, des = self.find_features(img)
            matches = self.match_features(prev_des, des)
            prev_p, p = self.filter_by_matches(prev_kp, kp, matches)

            # Find essential and filter
            E, mask = self.find_E(prev_p, p, K)
            E_matches = [match for match, inlier in zip(matches, mask.ravel() == 1) if inlier]
            prev_p, p = self.filter_by_matches(prev_kp, kp, E_matches)

            prev_p_h = np.hstack((prev_p, np.ones((prev_p.shape[0], 1))))
            p_h = np.hstack((p, np.ones((p.shape[0], 1))))
            l = prev_p_h @ E
            ds = np.zeros(l.shape[0])
            for i, ll in enumerate(l):
                ds[i] = ll @ p_h[i] / np.sqrt(ll[0] ** 2 + ll[1] ** 2)
            #print("mean:", np.mean(ds))
            #print("std:", np.std(ds))

            # Find common matches
            common_matches = []
            if 1 < idx:
                common_matches, new_matches = self.find_common_matches(prev_matches, E_matches)
            else:
                common_matches = []
                new_matches = E_matches

            # Do PnP thingy if more than 4 matches
            if 4 <= len(common_matches):
                common_p_3ds = np.array([map.points[map.observations[m2o[m.queryIdx]].point_id-uniqid].point for m in common_matches])[:, :3]
                common_ps = self.filter_by_matches(prev_kp, kp, common_matches)[0]
                _, R, t, _ = cv2.solvePnPRansac(
                    common_p_3ds,
                    common_ps,
                    K,
                    np.zeros(4)
                )
                R, _ = cv2.Rodrigues(R)
            else:
                R, t = self.get_pose(E, prev_p, p, K)

            # Add camera
            cam = Camera(idx, R, t)
            cam.pose = cam.pose @ map.cameras[-1].pose
            map.cameras.append(cam)

            # Triangulate points
            p_3d = self.triangulate(prev_p, p, K, map.cameras[-2].pose, cam.pose)

            if idx == 1:
                for i, pp in enumerate(prev_p):
                    map.observations.append(Observation(i + uniqid, 0, pp))

            m2o_new = {}
            for i, m in enumerate(E_matches):
                if m in new_matches:
                    idx3d = p_idx
                    map.points.append(Point(p_idx, p_3d[:, i]))#内点用point存
                    p_idx += 1
                else:
                    idx3d = map.observations[m2o[m.queryIdx]].point_id
                map.observations.append(Observation(idx3d, cam.id, p[i]))
                m2o_new[E_matches[i].trainIdx] = len(map.observations) - 1
            m2o = m2o_new

            prev_des = des
            prev_kp = kp
            prev_matches = E_matches

            if 1 < idx:
                removed = map.remove_camera(idx - 2)
                m2o = {k: v - removed for k, v in m2o.items()}

            # Calculate reprojection error
            print(f"{idx:}", map.reproj_err())
            map.bundle_adjustment()
            print(f"{idx:}", map.reproj_err())
            map.cameras[-1].fixed = True

            if 3 < idx:
                break


np.set_printoptions(precision=4, suppress=True)
if __name__ == "__main__":
    pl = Pipeline()
    pl.run("output/fifties")
else:
    # Task 9.1.1
    # coords_geo = extract_gps('input/DJIFlightRecord_2021-03-18_[13-04-51]-TxtLogToCsv.csv')
    # coords_utm = geodetic_to_utm(coords_geo)

    # Task 9.1.2
    # fi = FrameIterator('input/DJI_0199.MOV')
    # fi.main()

    # Task 9.2.1
    scale = 0.3
    img1 = cv2.imread("output/fifties/f20.jpg")
    img2 = cv2.imread("output/fifties/f21.jpg")
    img1 = cv2.resize(img1, (int(img1.shape[1] * scale), int(img1.shape[0] * scale)))
    img2 = cv2.resize(img2, (int(img2.shape[1] * scale), int(img2.shape[0] * scale)))
    f = 2676.1051390718389 * scale
    cx = -35.243952918157035 * scale + img1.shape[1] / 2
    cy = -279.58562078697361 * scale + img1.shape[0] / 2
    K = np.array([
        [f, 0, cx],
        [0, f, cy],
        [0, 0, 1]
    ])
    pl = Pipeline()

    kp1, des1 = pl.find_features(img1)
    kp2, des2 = pl.find_features(img2)

    # Task 9.2.2
    matches = pl.match_features(des1, des2)
    p1, p2 = pl.filter_by_matches(kp1, kp2, matches)

    E, mask = pl.find_E(p1, p2, K)
    E_matches = [match for match, inlier in zip(matches, mask.ravel() == 1) if inlier]
    p1, p2 = pl.filter_by_matches(kp1, kp2, E_matches)

    # 9.2.3
    p1_h = np.hstack((p1, np.ones((p1.shape[0], 1))))
    p2_h = np.hstack((p2, np.ones((p2.shape[0], 1))))
    l = p1_h @ E
    ds = np.zeros(l.shape[0])
    for i, ll in enumerate(l):
        ds[i] = ll @ p2_h[i] / np.sqrt(ll[0] ** 2 + ll[1] ** 2)
        print(ds[i])
    print("mean:", np.mean(ds))
    print("std:", np.std(ds))

    # 9.2.4
    R, t = pl.get_pose(E, p1, p2, K)
    cam = Camera(1, R, t)

    # 9.2.5
    p_3d = pl.triangulate(p1, p2, R, t, K)

    # 9.2.6
    map = Map(K)
    for i, pp in enumerate(p_3d.T):
        map.add_point(Point(i, pp))
    map.add(
        [Observation(i, i, 0, p) for i, p in enumerate(p1)],
        Camera(0)
    )
    map.add(
        [Observation(i, i, cam.id, p) for i, p in enumerate(p2)],
        cam
    )

    # img = cv2.drawMatches(img1, kp1, img2, kp2, inlier_matches, None, flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)
    # cv2.imshow("img", img)
    # cv2.waitKey()

    # 9.2.7
    print(map.reproj_err())
