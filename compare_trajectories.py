import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd


def load_only_coordinates_from_file(
    file_path: str,
) -> pd.DataFrame:
    return pd.read_csv(file_path)[['timestamp', 'tx', 'ty', 'tz']]


def distance_to_ground_truth(
    ground_truth: pd.DataFrame,
    result: pd.DataFrame,
) -> pd.DataFrame:
    return (ground_truth['tx'] - result['tx']) ** 2


def select_frames_from_ground_truth_by_index(
    gt: pd.DataFrame,
    other: pd.DataFrame,
) -> pd.DataFrame:
    return gt[gt.index.isin(other.timestamp)]


ground_truth_coordinates = pd.read_csv(
    '/home/george/Documents/datasets/dataset-room1_512_16/dso/gt_imu.csv'
)[['tx', 'ty', 'tz']]

old_pnp_xyz_coordinates = load_only_coordinates_from_file(
    file_path='OldPNPTrajectoryPath_key_frame.csv'
)

new_pnp_xyz_coordinates = load_only_coordinates_from_file(
    file_path='NewPNPTrajectoryPath_key_frame.csv'
)

frames_from_gt_for_new_pnp = select_frames_from_ground_truth_by_index(
    gt=ground_truth_coordinates,
    other=new_pnp_xyz_coordinates,
)
print(old_pnp_xyz_coordinates.head())
print(new_pnp_xyz_coordinates.head())


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot(xs=new_pnp_xyz_coordinates.tx, ys=new_pnp_xyz_coordinates.ty, zs=new_pnp_xyz_coordinates.timestamp, c='r')
ax.plot(xs=old_pnp_xyz_coordinates.tx, ys=old_pnp_xyz_coordinates.ty, zs=old_pnp_xyz_coordinates.timestamp, c='g')

plt.show(block=False)
