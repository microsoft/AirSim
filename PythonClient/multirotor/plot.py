"""
Module to plot the recorded drone data.

Args:
    --recording=<recording_folder_name_in_~/Documents/AirSim/>
    e.g. --recording=2022-04-19-14-57-09

"""
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from os.path import join, dirname, abspath, expanduser
from matplotlib.figure import Figure


def convert_unit(data) -> pd.DataFrame:
    data.TRUE_ANGLE_ROLL = data.TRUE_ANGLE_ROLL * 180 / np.pi
    data.TRUE_ANGLE_PITCH = data.TRUE_ANGLE_PITCH * 180 / np.pi
    data.TRUE_ANGLE_YAW = data.TRUE_ANGLE_YAW * 180 / np.pi
    data.EKF_ANGLE_ROLL = data.EKF_ANGLE_ROLL * 180 / np.pi
    data.EKF_ANGLE_PITCH = data.EKF_ANGLE_PITCH * 180 / np.pi
    data.EKF_ANGLE_YAW = data.EKF_ANGLE_YAW * 180 / np.pi
    data.EKF_ANGLE_ROLL_VAR = data.EKF_ANGLE_ROLL_VAR * 180 / np.pi * 180 / np.pi
    data.EKF_ANGLE_PITCH_VAR = data.EKF_ANGLE_PITCH_VAR * 180 / np.pi * 180 / np.pi
    data.EKF_ANGLE_YAW_VAR = data.EKF_ANGLE_YAW_VAR * 180 / np.pi * 180 / np.pi

    return data


def read_data_error(filename):
    data = pd.read_csv(filename, delimiter="\t", header=0)
    data = convert_unit(data)
    error = pd.DataFrame(data.TRUE_POS_X - data.EKF_POS_X, columns=["POS_X"])
    error = error.join(
        pd.DataFrame(data.TRUE_POS_Y - data.EKF_POS_Y, columns=["POS_Y"])
    )
    error = error.join(
        pd.DataFrame(data.TRUE_POS_Z - data.EKF_POS_Z, columns=["POS_Z"])
    )

    timestamp = data.TimeStamp
    start_time = timestamp[0]
    time = np.array(timestamp - start_time) * 0.001
    data = data.join(pd.DataFrame(time, columns=["Time"]))

    return (time, data, error)


def plot_with_confidence(ax, time, array, confidence, label, title) -> None:
    ax.plot(time, array[0], linestyle="solid", color="k", label=label[0])
    ax.legend()
    ax.plot(time, array[1], linestyle="solid", color="C0", label=label[1])
    ax.legend()
    ax.fill_between(
        time,
        array[1] - 3 * np.sqrt(confidence),
        array[1] + 3 * np.sqrt(confidence),
        alpha=0.2,
        linewidth=0,
        color="C5",
    )
    ax.set_ylabel(title)
    # ax.set_xlim([25, 30])
    # ax.set_ylim([2, 15])
    ax.grid(True)


def plot_with_confidence_error(ax, time, array, confidence, title, ylim=None) -> None:
    ax.plot(time, array, linestyle="solid", color="k")  # , label="error")
    # ax.legend()
    ax.fill_between(
        time,
        array - 3 * np.sqrt(confidence),
        array + 3 * np.sqrt(confidence),
        alpha=0.2,
        linewidth=0,
        color="C5",
    )
    ax.set_ylabel(title)
    # ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    ax.grid(True)


def plot_three_items(ax, time, array, label, title) -> None:
    ax.plot(time, array[0], linestyle="solid", color="C0", label=label[0])
    ax.legend()
    ax.plot(time, array[1], linestyle="dotted", color="C1", label=label[1])
    ax.legend()
    ax.plot(time, array[2], linestyle="solid", color="C2", label=label[2])
    ax.legend()
    ax.set_ylabel(title)
    ax.grid(True)


def make_figure(time, data, error) -> Figure:
    fig, ax = plt.subplots(nrows=26, ncols=1, figsize=(6, 50))

    idx = 0
    plot_three_items(
        ax[idx],
        time,
        (data.TRUE_POS_X, data.TRUE_POS_Y, data.TRUE_POS_Z),
        ["x", "y", "z"],
        "True Position(m)",
    )
    idx += 1
    plot_three_items(
        ax[idx],
        time,
        (data.EKF_POS_X, data.EKF_POS_Y, data.EKF_POS_Z),
        ["x", "y", "z"],
        "Estimated Position(m)",
    )
    idx += 1
    plot_three_items(
        ax[idx],
        time,
        (error.POS_X, error.POS_Y, error.POS_Z),
        ["x", "y", "z"],
        "Error Position(m)",
    )

    idx += 1
    plot_with_confidence_error(
        ax[idx],
        time,
        (error.POS_X),
        data.EKF_POS_X_VAR,
        "x Position Error(m)",
        ylim=[-2, 2]
        # ylim=[0,]
    )

    idx += 1
    plot_with_confidence_error(
        ax[idx],
        time,
        (error.POS_Y),
        data.EKF_POS_Y_VAR,
        "y Position Error(m)",
        ylim=[-2, 2],
    )
    idx += 1
    plot_with_confidence(
        ax[idx],
        time,
        (data.TRUE_POS_Z, data.EKF_POS_Z),
        data.EKF_POS_Z_VAR,
        ["true", "est"],
        "z Position(m)",
    )
    idx += 1
    plot_with_confidence(
        ax[idx],
        time,
        (data.TRUE_ANGLE_ROLL, data.EKF_ANGLE_ROLL),
        data.EKF_ANGLE_ROLL_VAR,
        ["true", "est"],
        "Roll(deg)",
    )
    idx += 1
    plot_with_confidence(
        ax[idx],
        time,
        (data.TRUE_ANGLE_PITCH, data.EKF_ANGLE_PITCH),
        data.EKF_ANGLE_PITCH_VAR,
        ["true", "est"],
        "Pitch(deg)",
    )
    idx += 1
    plot_with_confidence(
        ax[idx],
        time,
        (data.TRUE_ANGLE_YAW, data.EKF_ANGLE_YAW),
        data.EKF_ANGLE_YAW_VAR,
        ["true", "est"],
        "Yaw(deg)",
    )
    idx += 1
    plot_with_confidence(
        ax[idx],
        time,
        (data.TRUE_VEL_X, data.EKF_VEL_X),
        data.EKF_VEL_X_VAR,
        ["true", "est"],
        "x Vel(m/s)",
    )
    idx += 1
    plot_with_confidence(
        ax[idx],
        time,
        (data.TRUE_VEL_Y, data.EKF_VEL_Y),
        data.EKF_VEL_Y_VAR,
        ["true", "est"],
        "y Vel(m/s)",
    )
    idx += 1
    plot_with_confidence(
        ax[idx],
        time,
        (data.TRUE_VEL_Z, data.EKF_VEL_Z),
        data.EKF_VEL_Z_VAR,
        ["true", "est"],
        "z Vel(m/s)",
    )
    idx += 1
    ax[idx].plot(time, data.QUAT_NORM, linestyle="solid", color="C2")
    ax[idx].set_ylabel("Quat Norm")
    ax[idx].grid(True)

    idx += 1
    plot_with_confidence_error(
        ax[idx], time, (data.BIAS_ACCEL_Y), data.BIAS_ACCEL_Y_VAR, "x bias accel(m/s2)"
    )
    idx += 1
    plot_with_confidence_error(
        ax[idx], time, (data.BIAS_ACCEL_Y), data.BIAS_ACCEL_Y_VAR, "y bias accel(m/s2)"
    )
    idx += 1
    plot_with_confidence_error(
        ax[idx], time, (data.BIAS_ACCEL_Z), data.BIAS_ACCEL_Z_VAR, "z bias accel(m/s2)"
    )
    idx += 1
    plot_with_confidence_error(
        ax[idx],
        time,
        (data.BIAS_GYRO_Y * 180 * np.pi),
        data.BIAS_GYRO_Y_VAR * 180 * np.pi * 180 * np.pi,
        "x bias gyro(deg/s)",
    )
    idx += 1
    plot_with_confidence_error(
        ax[idx],
        time,
        (data.BIAS_GYRO_Y * 180 * np.pi),
        data.BIAS_GYRO_Y_VAR * 180 * np.pi * 180 * np.pi,
        "y bias gyro(deg/s)",
    )
    idx += 1
    plot_with_confidence_error(
        ax[idx],
        time,
        (data.BIAS_GYRO_Z * 180 * np.pi),
        data.BIAS_GYRO_Z_VAR * 180 * np.pi * 180 * np.pi,
        "z bias gyro(deg/s)",
    )
    idx += 1
    plot_with_confidence_error(
        ax[idx], time, (data.BIAS_BARO), data.BIAS_BARO_VAR, "bias baro(m)"
    )

    idx += 1
    plot_three_items(
        ax[idx],
        time,
        (data.ACCEL_X, data.ACCEL_Y, data.ACCEL_Z),
        ["x", "y", "z"],
        "accel(m/s2)",
    )
    idx += 1
    plot_three_items(
        ax[idx],
        time,
        (
            data.GYRO_X * 180 * np.pi,
            data.GYRO_Y * 180 * np.pi,
            data.GYRO_Z * 180 * np.pi,
        ),
        ["x", "y", "z"],
        "gyro(deg/s)",
    )
    idx += 1
    plot_three_items(
        ax[idx],
        time,
        (data.GPS_POS_X, data.GPS_POS_Y, data.GPS_POS_Z),
        ["x", "y", "z"],
        "gps pos(m)",
    )
    idx += 1
    plot_three_items(
        ax[idx],
        time,
        (data.GPS_VEL_X, data.GPS_VEL_Y, data.GPS_VEL_Z),
        ["x", "y", "z"],
        "gps vel(m/s)",
    )
    idx += 1
    plot_three_items(
        ax[idx],
        time,
        (data.MAG_X, data.MAG_Y, data.MAG_Z),
        ["x", "y", "z"],
        "mag flux(Gauss)",
    )
    idx += 1
    ax[idx].plot(time, data.BARO_ALT, linestyle="solid", color="C2")
    ax[idx].set_ylabel("baro alt(m)")
    ax[idx].grid(True)
    ax[idx].set_xlabel("Time(s)")

    return fig



def read_and_plot(recording: str, path) -> pd.DataFrame:
    plt.style.use(join(path, "style.mplstyle"))
    filename = join("~/Documents/AirSim", recording, "airsim_rec.txt")
    print(f"Plotting the recorded data: {filename}")

    (time, data, error) = read_data_error(filename)
    fig = make_figure(time, data, error)

    fig_name = join(expanduser("~/Documents/AirSim"), recording, "figure.pdf")
    fig.savefig(fig_name)
    print(f"Plot saved in: {fig_name}")


def main() -> None:
    """Main entry point"""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--recording", help="name of the recording folder", required=True
    )
    args = parser.parse_args()

    path = dirname(abspath(__file__))

    read_and_plot(args.recording, path)

if __name__ == "__main__":
    main()
