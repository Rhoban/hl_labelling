#!/usr/bin/env python3

import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

CAM_HEIGHT_ROBOT = 0.68
colors = {0:"DarkBlue", 1:"red"}

first_annotation_index = -1 # should be the same for all entries
plot_registry = dict()

def plot(func):
    plot_registry[func.__name__] = func

def draw3D(df):
    ax = plt.axes(projection='3d')
    ax.set_xlim((-5,5))
    ax.set_ylim((-4,4))
    grouped = df.groupby('is_annotation_frame')
    for key, group in grouped:
        if not key:
            continue
        ax.plot(ax=ax,
                x='camera_estimate_translation_x',
                y='camera_estimate_translation_y',
                z='camera_estimate_translation_z',
                c=group["is_annotation_frame"].map(colors),
                label="image_annotée" if key else "image_non_annotée",
                edgecolor=colors[key],
                title="Roll over time")
    # ax.plot(df["camera_estimate_translation_x"],
    #         df["camera_estimate_translation_y"],
    #         df["camera_estimate_translation_z"],
    #         label="translation camera"
    #        )


# ORIENTATION
def drawRoll(df):
   grouped = df.groupby('is_annotation_frame')
   for key, group in grouped:
       print(key)
       group.plot(ax=ax, kind='scatter',
                   x='time (s)',
                   y='camera_estimate_roll',
                   c=group["is_annotation_frame"].map(colors), label="image_annotée" if key else "image_non_annotée", edgecolor=colors[key], title="Roll over time")
   # f = plt.gcf()
   # cax = f.get_axes()[0]
   # cax.axhline(y=CAM_HEIGHT_ROBOT, c='r', linestyle=":")

def drawField(color="green"):
   plt.axline((-4.5, 3), (4.5,3), c=color)
   plt.axline((-4.5, -3), (4.5,-3), c=color)
   plt.axline((-4.5, 3), (-4.5,-3), c=color)
   plt.axline((4.5, 3), (4.5,-3), c=color)
   plt.axline((0, 3), (0,-3), c=color)

   # plt.axhline(y=-4.5, color=color, linestyle='-')
   # plt.axhline(y=4.5, color=color, linestyle='-')
   # plt.plot([0,-4.5]4.5, 3), (4.5,-3), c=color)

@plot
def refPosition2D(ref, estimations, diff_ref_from_one):
   fig, ax = plt.subplots()

   ax.set_xlim((-5,5))
   ax.set_ylim((-4,4))
   drawField()

   sc = ref[ ref.is_annotation_frame == 0].plot.scatter(x='camera_estimate_translation_x',
                                                        y='camera_estimate_translation_y',
                                                        ax=ax,
                                                        c="time (s)",
                                                        colormap='viridis',
                                                        colorbar=True,
                                                        label="images non étiquetées" )

   sc = ref[ ref.is_annotation_frame == 1].plot.scatter(x='camera_estimate_translation_x',
                                                        y='camera_estimate_translation_y',
                                                        ax=ax,
                                                        c="time (s)",
                                                        edgecolor="red",
                                                        colormap='viridis',
                                                        colorbar=False,
                                                        label="images étiquetées" )
   # customize colorbar
   cax = fig.get_axes()[1]
   for k, row in ref.iterrows():
       if row.is_annotation_frame == 1:
           # cax.axhline(y=row.time_s, xmin=0, xmax=0.1,c='r')
           # cax.axhline(y=row.time_s, xmin=0.9, xmax=1,c='r')
           cax.axhline(y=row["time (s)"],c='r', linestyle='-.', linewidth=1)
   plt.draw()

@plot
def estimatePosition2D(ref, estimations, diff_ref_from_one):
   fig, ax = plt.subplots()

   ax.set_xlim((-5,5))
   ax.set_ylim((-4,4))
   drawField()

   sc = ref[ ref.is_annotation_frame == 0].plot.scatter(x='camera_estimate_translation_x',
                                                        y='camera_estimate_translation_y',
                                                        ax=ax,
                                                        c="time (s)",
                                                        colorbar=True,
                                                        label="images non étiquetées ref" )

   sc = ref[ ref.is_annotation_frame == 1].plot.scatter(x='camera_estimate_translation_x',
                                                        y='camera_estimate_translation_y',
                                                        ax=ax,
                                                        c="time (s)",
                                                        edgecolor="purple",
                                                        colorbar=False,
                                                        label="images étiquetées ref" )

   cax = fig.get_axes()[1]
   for k, row in ref.iterrows():
       if row.is_annotation_frame == 1:
           # cax.axhline(y=row.time_s, xmin=0, xmax=0.1,c='r')
           # cax.axhline(y=row.time_s, xmin=0.9, xmax=1,c='r')
           cax.axhline(y=row["time (s)"],c='purple', linestyle='-.', linewidth=1)

   sc = estimations[0][ estimations[0].is_annotation_frame == 0].plot.scatter(x='camera_estimate_translation_x',
                                                        y='camera_estimate_translation_y',
                                                        ax=ax,
                                                        c="time (s)",
                                                        colormap='viridis',
                                                        colorbar=True,
                                                        label="images non étiquetées" )

   sc = estimations[0][ estimations[0].is_annotation_frame == 1].plot.scatter(x='camera_estimate_translation_x',
                                                        y='camera_estimate_translation_y',
                                                        ax=ax,
                                                        c="time (s)",
                                                        edgecolor="red",
                                                        colormap='viridis',
                                                        colorbar=False,
                                                        label="images étiquetées" )
   # customize colorbar
   cax = fig.get_axes()[2]
   for k, row in estimations[0].iterrows():
       if row.is_annotation_frame == 1:
           # cax.axhline(y=row.time_s, xmin=0, xmax=0.1,c='r')
           # cax.axhline(y=row.time_s, xmin=0.9, xmax=1,c='r')
           cax.axhline(y=row["time (s)"],c='r', linestyle='-.', linewidth=1)
   plt.draw()

def diff(ref, estimations, column_name, custom_label="", x_axis="time (s)", fig=None, ax=None):
    is_subplot = True
    if fig == None and ax == None:
        fig, ax = plt.subplots()
        is_subplot = False

    name = "diff_" + column_name if custom_label == "" else custom_label

    # diff_z["camera_estimate_translation_z"] = ref["camera_estimate_translation_z"] - estimations[0]["camera_estimate_translation_z"]
    estimations[0][name] = estimations[0][column_name] - ref[column_name]

    sc = estimations[0][estimations[0].is_annotation_frame == 1].plot.scatter(x=x_axis,
                                                                              y=name,
                                                                              ax=ax, c="red",
                                                                              subplots=is_subplot,
                                                                              label="images étiquetées" )
    sc = estimations[0][estimations[0].is_annotation_frame == 0].plot.scatter(x=x_axis,
                                                                              y=name,
                                                                              ax=ax,
                                                                              c="darkblue",
                                                                              subplots=is_subplot,
                                                                              label="images non étiquetées" )
    return fig, ax


@plot
def refZ(ref, estimations, diff_ref_from_one):
    fig, ax = plt.subplots()
    sc = ref[ref.is_annotation_frame == 1].plot.scatter(x="time (s)",
                                                        y="camera_estimate_translation_z",
                                                        ax=ax, c="red",
                                                        label="images étiquetées" )
    sc = ref[ref.is_annotation_frame == 0].plot.scatter(x="time (s)",
                                                        y="camera_estimate_translation_z",
                                                        ax=ax,
                                                        c="darkblue",
                                                        label="images non étiquetées" )


@plot
def diffX(ref, estimations, diff_ref_from_one):
   fig, ax = diff(ref, estimations, "camera_estimate_translation_x")

   # reference_min = data_frames[0].copy()
   # reference_max = data_frames[0].copy()
   # for i, df in enumerate(data_frames[:-1]):
   #       # reference_min['camera_estimate_translation_z'] = np.where(df['camera_estimate_translation_z'] < reference_min['camera_estimate_translation_z'], df['camera_estimate_translation_z'], reference_min['camera_estimate_translation_z'])
   #       # reference_max['camera_estimate_translation_z'] = np.where(df['camera_estimate_translation_z'] > reference_min['camera_estimate_translation_z'], df['camera_estimate_translation_z'], reference_max['camera_estimate_translation_z'])
   #     for k, row in df.iterrows():
   #         # print(row["camera_estimate_translation_z"])
   #         if reference_min.at[k,"camera_estimate_translation_z"] > row["camera_estimate_translation_z"]:
   #           reference_min["camera_estimate_translation_z"] = row["camera_estimate_translation_z"]

   #         if reference_max.at[k,"camera_estimate_translation_z"] < row["camera_estimate_translation_z"]:
   #           reference_max.at[k, "camera_estimate_translation_z"] = row["camera_estimate_translation_z"]
   # plt.plot(data_frames[-1]["time (s)"],
   #          data_frames[-1]["camera_estimate_translation_z"], 'or')
   # plt.fill_between(estimations[0]["time (s)"],
   #                  estimations[0]["camera_estimate_translation_x"] - 0.1,
   #                  estimations[0]["camera_estimate_translation_x"] +  0.1,
   #                   color='gray', alpha=0.2)

@plot
def diffY(ref, estimations):
   fig, ax = diff(ref, estimations, "camera_estimate_translation_y")
@plot
def diffZ(ref, estimations, diff_ref_from_one):
   fig, ax = diff(ref, estimations, "camera_estimate_translation_z")

@plot
def diffXY(ref, estimations, diff_ref_from_one):
   fig, ax = plt.subplots(nrows=1, ncols=1)

   number = len(estimations)
   cmap = plt.get_cmap('gnuplot')
   colors = [cmap(i) for i in np.linspace(0, 1, number)]

   for i, estimate in enumerate(estimations):
       estimate["distance_from_ref"] = np.sqrt( np.square(ref["camera_estimate_translation_x"] - estimate["camera_estimate_translation_x"]) + np.square(ref["camera_estimate_translation_y"] - estimate["camera_estimate_translation_y"]))
       # plt.plot(estimate[ estimate.is_annotation_frame == 0]["time_from_last_annotation"], estimate[ estimate.is_annotation_frame == 0]["distance_from_ref"], 'or', label=estimate.name[0])
       sc = estimate[ estimate.is_annotation_frame == 0].plot.scatter(x='time (s)',
                                                                      y="distance_from_ref",
                                                                      # ax=ax[2],
                                                                      ax=ax,
                                                                      c=colors[i],
                                                                      subplots=True,
                                                                      label=estimate.name
                                                                  )
       for v in estimate[ estimate.is_annotation_frame == 1]["time (s)"]:
            ax.axvline(x=v, color=colors[i], linestyle='-')
       # sc = estimate[ estimate.is_annotation_frame == 1].plot.scatter(x='time (s)',
       #                                                                y="distance_from_ref",
       #                                                                # ax=ax[2],
       #                                                                ax=ax,
       #                                                                c=colors[i],
       #                                                                subplots=True,
       #                                                                label=estimate.name
       #                                                            )

   # diff(ref, estimations, "camera_estimate_translation_x", custom_label="diff_x (s)", fig=fig, ax=ax[0]  )
   # diff(ref, estimations, "camera_estimate_translation_y", custom_label="diff_y (s)", fig=fig, ax=ax[1]  )


   # estimate = estimations[0]
   # estimate["distance_from_ref"] = np.sqrt( np.square(ref["camera_estimate_translation_x"] - estimate["camera_estimate_translation_x"]) + np.square(ref["camera_estimate_translation_y"] - estimate["camera_estimate_translation_y"]))
   # sc = estimate[ estimate.is_annotation_frame == 0].plot.scatter(x='time (s)',
   #                                                                y="distance_from_ref",
   #                                                                ax=ax[2],
   #                                                                c="darkblue",
   #                                                                subplots=True,
   #                                                            )
@plot
def diffXY_from_distance(ref, estimations, diff_ref_from_one):
   fig, ax = plt.subplots(nrows=1, ncols=1)

   number = len(estimations)
   cmap = plt.get_cmap('gnuplot')
   colors = [cmap(i) for i in np.linspace(0, 1, number)]

   for i, estimate in enumerate(estimations):
       estimate["distance_from_ref"] = np.sqrt( np.square(ref["camera_estimate_translation_x"] - estimate["camera_estimate_translation_x"]) + np.square(ref["camera_estimate_translation_y"] - estimate["camera_estimate_translation_y"]))
       # plt.plot(estimate[ estimate.is_annotation_frame == 0]["time_from_last_annotation"], estimate[ estimate.is_annotation_frame == 0]["distance_from_ref"], 'or', label=estimate.name[0])
       sc = estimate[ estimate.is_annotation_frame == 0].plot.scatter(x='time_from_last_annotation',
                                                                      y="distance_from_ref",
                                                                      ax=ax,
                                                                      c=colors[i],
                                                                      subplots=True,
                                                                      label=estimate.name
                                                                  )

@plot
def diffXY_relative_to_one(ref, estimations, diff_ref_from_one):
   fig, ax = plt.subplots(nrows=1, ncols=1)
   estimate = estimations[0]

   diff_ref_from_one["distance_from_ref"] = np.sqrt( np.square(ref["camera_estimate_translation_x"] - diff_ref_from_one["camera_estimate_translation_x"]) + np.square(ref["camera_estimate_translation_y"] - diff_ref_from_one["camera_estimate_translation_y"]))

   estimate["distance_from_ref"] = np.sqrt( np.square(ref["camera_estimate_translation_x"] - estimate["camera_estimate_translation_x"]) + np.square(ref["camera_estimate_translation_y"] - estimate["camera_estimate_translation_y"]))


   estimate["diff_distance_from_ref"] = estimate["distance_from_ref"] - diff_ref_from_one["distance_from_ref"]


   sc = estimate[ estimate.is_annotation_frame == 0].plot.scatter(x='time_from_last_annotation',
                                                                  y="diff_distance_from_ref",
                                                                  ax=ax,
                                                                  c="darkblue",
                                                                  subplots=True)

@plot
def diffX_relative_to_one(ref, estimations, diff_ref_from_one):
   fig, ax = plt.subplots(nrows=1, ncols=1)

   estimations[0]["camera_estimate_translation_x"] = estimations[0]["camera_estimate_translation_x"] - ref["camera_estimate_translation_x"]
   diff(diff_ref_from_one, estimations, "camera_estimate_translation_x", custom_label="diff_x_relativ (m)", fig=fig, ax=ax)

   # diff(diff_ref_from_one, estimations, "camera_estimate_translation_x", custom_label="diff_x_relativ (m)", fig=fig, ax=ax)

@plot
def diffXYZ_relative_to_one(ref, estimations, diff_ref_from_one):
   fig, ax = plt.subplots(nrows=3, ncols=1)

   estimations[0]["camera_estimate_translation_x"] = estimations[0]["camera_estimate_translation_x"] - ref["camera_estimate_translation_x"]
   estimations[0]["camera_estimate_translation_y"] = estimations[0]["camera_estimate_translation_y"] - ref["camera_estimate_translation_y"]
   estimations[0]["camera_estimate_translation_z"] = estimations[0]["camera_estimate_translation_z"] - ref["camera_estimate_translation_z"]

   diff(diff_ref_from_one, estimations, "camera_estimate_translation_x", custom_label="diff_x_relativ (m)", fig=fig, ax=ax[0])
   diff(diff_ref_from_one, estimations, "camera_estimate_translation_y", custom_label="diff_y_relativ (m)", fig=fig, ax=ax[1])
   diff(diff_ref_from_one, estimations, "camera_estimate_translation_z", custom_label="diff_z_relativ (m)", fig=fig, ax=ax[2])

@plot
def diffRPY(ref, estimations, diff_ref_from_one):
   fig, ax = plt.subplots(nrows=3, ncols=1)

   diff(ref, estimations, "camera_estimate_roll", custom_label="diff_roll", fig=fig, ax=ax[0]  )
   diff(ref, estimations, "camera_estimate_pitch", custom_label="diff_pitch", fig=fig, ax=ax[1]  )
   diff(ref, estimations, "camera_estimate_yaw", custom_label="diff_yaw", fig=fig, ax=ax[2]  )

   for p in ax:
       p.axhline(y=0,color='black', linestyle='-')
@plot
def diffRPY_relative_to_one(ref, estimations, diff_ref_from_one):
   fig, ax = plt.subplots(nrows=3, ncols=1)

   estimations[0]["camera_estimate_roll"] = estimations[0]["camera_estimate_roll"] - ref["camera_estimate_roll"]
   estimations[0]["camera_estimate_pitch"] = estimations[0]["camera_estimate_pitch"] - ref["camera_estimate_pitch"]
   estimations[0]["camera_estimate_yaw"] = estimations[0]["camera_estimate_yaw"] - ref["camera_estimate_yaw"]

   diff(diff_ref_from_one, estimations, "camera_estimate_roll", custom_label="diff_roll", fig=fig, ax=ax[0]  )
   diff(diff_ref_from_one, estimations, "camera_estimate_pitch", custom_label="diff_pitch", fig=fig, ax=ax[1]  )
   diff(diff_ref_from_one, estimations, "camera_estimate_yaw", custom_label="diff_yaw", fig=fig, ax=ax[2]  )

   for p in ax:
       p.axhline(y=0,color='black', linestyle='-')


def loadDataFrame(paths):
    array = []
    for csv_path in paths:
        df = pd.read_csv(csv_path)
        array.append(df)
    return array


last_annotation_time = 0
def lambda_time_from_last_annotation(current_time, is_annotation_frame):
    global last_annotation_time
    if is_annotation_frame:
        last_annotation_time = current_time
    return current_time - last_annotation_time

def preprocess(df,
               start_at_0:bool = True,
               time_s_column:bool = True,
               remove_data_until_first_label:bool=True,
               time_from_last_annotation:bool=True):
    global first_annotation_index

    if start_at_0:
        first_time_us = df["time_us"][0]
        df["time_us"] = df["time_us"].apply(lambda x: x - first_time_us)

    if time_s_column:
        df["time (s)"] = df["time_us"].apply(lambda x: x / 1000000)

    if remove_data_until_first_label:
        current_first_annotation = df[ df.is_annotation_frame == 1]["frame_id"].iloc[0]
        if first_annotation_index == -1:
            first_annotation_index = current_first_annotation
        elif first_annotation_index != current_first_annotation:
            print(f"diff annotation index: required: {first_annotation_index} actual: {k}")
            return False

        df.drop(np.arange(0,first_annotation_index), inplace=True)
        df.reset_index(drop=True, inplace=True)

    if time_from_last_annotation:
        df["time_from_last_annotation"] = df.apply(lambda x : lambda_time_from_last_annotation(x["time (s)"], x["is_annotation_frame"]), axis=1)

    return True


class Format:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Diff tool")
    parser.add_argument('--refs', help="reference trajectories", nargs='+',
                        metavar='file1.csv file2.csv', type=str)
    parser.add_argument('--ref_1_label', help="reference trajectory with one label", metavar='traj_with_one_label.csv', type=str, required=True)
    parser.add_argument('--estimations', help="estimated trajectories", nargs='+', metavar='file1.csv file2.csv', type=str, required=True)
    parser.add_argument('--plot', dest='plot', help="choose a plot to display", type=str, choices=plot_registry.keys(), required=True)
    args = parser.parse_args()

    # gloabal plot settings
    plt.rcParams.update({'font.size': 22})

    # load
    refs = loadDataFrame(args.refs)
    estimations = loadDataFrame(args.estimations)
    ref_one_label = pd.read_csv(args.ref_1_label)

    # preprocess
    preprocess(ref_one_label)
    ref_one_label.name = args.ref_1_label
    for i, df in enumerate(refs):
        df.name = args.refs[i]
        if not preprocess(df):
           print(f"{Format.FAIL}ERROR{Format.ENDC}: all entries should start with the same annotation: {Format.BOLD}{Format.WARNING}{args.refs[i]}{Format.ENDC} invalid")
           exit(1)
    for i, df in enumerate(estimations):
        df.name = args.estimations[i]
        if not preprocess(df):
           print(f"{Format.FAIL}ERROR{Format.ENDC}: all entries should start with the same annotation: {Format.BOLD}{Format.WARNING}{args.estimations[i]}{Format.ENDC} invalid")
           exit(1)

    # data extraction : refs_mean, diff_refs_mean_and_one_label
    refs_mean = refs[0].copy()
    for head in ("camera_estimate_translation_x",
                 "camera_estimate_translation_y",
                 "camera_estimate_translation_z",
                 "camera_estimate_roll",
                 "camera_estimate_pitch",
                 "camera_estimate_yaw",
                ):
        for r in refs[1:]:
            refs_mean[head] = refs_mean[head] + r[head]
        refs_mean[head] = refs_mean[head] / len(refs)

    diff_refs_mean_and_one_label = refs_mean.copy()
    for head in ("camera_estimate_translation_x",
                 "camera_estimate_translation_y",
                 "camera_estimate_translation_z",
                 "camera_estimate_roll",
                 "camera_estimate_pitch",
                 "camera_estimate_yaw",
                ):
        diff_refs_mean_and_one_label[head] = diff_refs_mean_and_one_label[head] - ref_one_label[head]

    # argparse ensure that args.plot is valid
    plot_registry[args.plot](refs_mean, estimations, diff_refs_mean_and_one_label)
    plt.show()
