# -*- coding: utf-8 -*-
"""
Tool for yaml-based automatic report generation from logged data of the crazyflie.
"""

# attidtue best: 22

import cfusdlog
import matplotlib.pyplot as plt
import os
import sys
import yaml
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np


import model


def compute_tracking_error(data, settings):
    # extract the data for the position
    actual_x = data[settings["event_name"]]["stateEstimateZ.x"]
    actual_y = data[settings["event_name"]]["stateEstimateZ.y"]
    actual_z = data[settings["event_name"]]["stateEstimateZ.z"]
    desired_x = data[settings["event_name"]]["ctrltargetZ.x"]
    desired_y = data[settings["event_name"]]["ctrltargetZ.y"]
    desired_z = data[settings["event_name"]]["ctrltargetZ.z"]
        
    # compute the L2 vector for the position
    traj_actual = np.array([actual_x, actual_y, actual_z])
    traj_desired = np.array([desired_x, desired_y, desired_z])
    traj_delta = traj_actual - traj_desired
    traj_sq = traj_delta.T@traj_delta
    l2_vec_position = np.sqrt(np.diag(traj_sq))

    # extract the data for the roll
    actual_roll = data[settings["event_name"]]["ctrlLee.rpyx"]
    desired_roll = data[settings["event_name"]]["ctrlLee.rpydx"]

    # compute the L2 vector for the position
    angle_actual = np.array([actual_roll])
    # print(angle_actual.shape)
    angle_desired = np.array([desired_roll])
    angle_delta = angle_actual - angle_desired
    angle_sq = angle_delta.T@angle_delta
    l2_vec_angle = np.sqrt(np.diag(angle_sq))

    e_dict = {}
    for error in settings["errors"]:
        e_dict[error] = -1

        if error == "L2 integral error":
            e_position = 0
            e_angle = 0
            for i in range(1, len(l2_vec_position)):
                t_delta = data[settings["event_name"]]['timestamp'][i] - data[settings["event_name"]]['timestamp'][i-1]
                l2_delta_position = l2_vec_position[i] - l2_vec_position[i-1]
                l2_delta_angle = l2_vec_angle[i] - l2_vec_angle[i-1]
                e_position += l2_delta_position*t_delta
                e_angle += l2_delta_angle*t_delta

            e_dict[error] = f"{np.round(e_position*1e3, 3)}mm; {np.round(e_angle, 3)}deg"
                
        elif error == "L2 mean":
            e_dict[error] = f"{np.round(np.mean(l2_vec_position)*1e3, 3)}mm; {np.round(np.mean(l2_vec_angle), 3)}deg"
        elif error == "L2 std":
            e_dict[error] = f"{np.round(np.std(l2_vec_position)*1e3, 3)}mm; {np.round(np.std(l2_vec_angle), 3)}deg"
        elif error == "L2 max":
            timepoint = (data[settings["event_name"]]['timestamp'][np.argmax(l2_vec_position)] - data[settings["event_name"]]['timestamp'][0]) / 1000
            e_dict[error] = f"{np.round(np.max(l2_vec_position)*1e3)}mm@{np.round(timepoint, 3)}s"

            timepoint = (data[settings["event_name"]]['timestamp'][np.argmax(l2_vec_angle)] - data[settings["event_name"]]['timestamp'][0]) / 1000
            e_dict[error] += f"; {np.round(np.max(l2_vec_angle))}deg@{np.round(timepoint, 3)}s"

    return e_dict

def file_guard(pdf_path):
    msg = None
    if os.path.exists(pdf_path):
        msg = input("file already exists, overwrite? [y/n]: ")
        if msg == "n":
            print("exiting...")
            sys.exit(0)
        elif msg == "y":
            print("overwriting...")
            os.remove(pdf_path)
        else:
            print("invalid msg...")
            file_guard(pdf_path)

    return


def process_data(data, settings):
    print("...processing data")

    # adjust time
    start_time = settings["start_time"]
    # end_time = settings["end_time"]
    event = settings["event_name"]

    # convert units
    if settings["convert_units"]:
        for key, value in settings["convert_units"].items():
            data[event][key] = data[event][key] * value

    # shift timestamp data
    if start_time is None:
        start_time = data[event]['timestamp'][0]
    else:
        start_time = min(start_time, data[event]['timestamp'][0])

    data[event]["timestamp"] = (data[event]["timestamp"] - start_time)

    # add additional data to the data dictionary)
    add_data(data, settings)

    # print(data[event].keys())
    # print(data[event].items())

    return data


def add_data(data, settings):
    event = settings["event_name"]
    print("...adding data")
    
    for info in settings["additional_data"]:
        # print(f"found target: {info['target']}")
        name, data_new = model.DataHelper.generate_data(data, event, info)
        data[event][name] = data_new
        print(f">>> added data: {name} ({info['type']})")
        # print(f">>> data shape: {data_new.shape}")

    print("...done adding data")


def create_figures(data_usd, settings, log_str):
    debug_all = False
    debug = False
    debug_figure_number = 6 # payload positions
    # debug_figure_number = 7 # payload velocities


    log_path = os.path.join(settings["data_dir"], log_str)
    print("log file: {}".format(log_path))

    data_processed = process_data(data_usd, settings)

    # create a PDF to save the figures
    pdf_path =  os.path.join(settings["output_dir"], log_str) + ".pdf"
    print("output path: {}".format(pdf_path))

    # check if user wants to overwrite the report file
    file_guard(pdf_path)

    pdf_pages = PdfPages(pdf_path)

    # create the title page
    title_text_settings = f"Settings:\n"
    for setting in settings["title_settings"]:
        title_text_settings += f"    {setting}: {settings[setting]}\n"

    # read the parameters from the info file
    info_file = f"{settings['info_dir']}/{log_str.replace('log', 'info').split('_')[0]}.yaml" 
    print("info file: {}".format(info_file))

    try:
        with open(info_file, "r") as f:
            info = yaml.safe_load(f)
    except FileNotFoundError:
        print(f"File not found: {info_file}")
        exit(1)

    title_text_parameters = f"Parameters:\n"
    for key, value in info.items():
        title_text_parameters += f"    {key}: {value}\n"

    # create the results section
    title_text_results = f"Results:\n"
    e_dict = compute_tracking_error(data_processed, settings)
    for error in settings["errors"]:
        title_text_results += f"    {error}: {e_dict[error]}\n"

    text = f"%% Lee controller tuning %%\n"
    title_text = text + "\n" + title_text_settings + "\n" + title_text_parameters + "\n" + title_text_results
    fig = plt.figure(figsize=(5, 8))
    fig.text(0.1, 0.1, title_text, size=11)
    pdf_pages.savefig(fig)

    # create data plots
    figures_max = settings.get("figures_max", None)  # set to None to plot all figures
    figure_count = 0
    for k, (event, data) in enumerate(data_processed.items()):
        if event in settings["event_name"]:
            print("processing event: {} ({})".format(event, k))

            # create a new figure for each value in the data dictionary
            for figure_info in settings["figures"]:
                if figures_max is not None and figure_count >= figures_max:
                    break

                title = figure_info["title"]
                figure_type = figure_info["type"]
                x_label = figure_info.get("x_label", None)
                y_label = figure_info.get("y_label", None)
                z_label = figure_info.get("z_label", None)
                structure = figure_info["structure"]
                structure_length = len(structure)
                
                if figure_type == "2d subplots":
                    fig, ax = plt.subplots(structure_length, 1)

                    if structure_length == 1:
                        ax = [ax]
                    
                    # iterate over every subplot in the figure
                    for i, obj in enumerate(structure):
                        n_x = len(obj["x_axis"])
                        n_y = len(obj["y_axis"])
                        n_leg = len(obj["legend"])

                        if n_x != n_y != n_leg:
                            raise ValueError("Please specify the same number of x and y signals and legends")
                        
                        # iterate over every plot in the respective subplot
                        for j in range(n_x):
                            x = obj["x_axis"][j]
                            y = obj["y_axis"][j]

                            if figure_info["marker"] == "line":
                                ax[i].plot(data[x], data[y], label=obj["legend"][j], **figure_info["marker_kwargs"])
                            elif figure_info["marker"] == "scatter":
                                ax[i].scatter(data[x], data[y], label=obj["legend"][j], **figure_info["marker_kwargs"])
                            else:
                                raise ValueError("Invalid marker")

                            ax[i].set_xlabel(obj["x_label"])
                            ax[i].set_ylabel(obj["y_label"])
                            ax[i].legend(loc="lower left", fontsize=5)
                            ax[i].grid(True)

                # DEPRECATED
                # if figure_type == "2d single":
                #     fig, ax = plt.subplots()
                    
                #     # iterate over every subplot
                #     for obj in structure:
                #         ax.plot(data[obj["x_axis"]], 
                #             data[obj["y_axis"]], 
                #             label=obj["legend"], 
                #             linewidth=0.5)
                #         ax.set_xlabel(obj["x_label"])
                #         ax.set_ylabel(obj["y_label"])
                #         ax.legend(loc="lower left", fontsize=5)
                #         ax.grid(True)

                if figure_type == "3d":
                    fig = plt.figure()
                    ax = fig.add_subplot(projection='3d')

                    y_label = figure_info["y_label"]
                    
                    # iterate over every subplot
                    for i, obj in enumerate(structure):
                        ax.plot(data[obj[0]],
                                data[obj[1]],
                                data[obj[2]], 
                                label=obj[3], 
                                linewidth=0.5)
                        
                        ax.set_xlim(min(data[obj[0]])-0.1*min(data[obj[0]]), 
                                    max(data[obj[0]])+0.1*max(data[obj[0]]))
                        ax.set_ylim(min(data[obj[1]])-0.1*min(data[obj[1]]),
                                    max(data[obj[1]])+0.1*max(data[obj[1]]))
                        ax.set_zlim(min(data[obj[2]])-0.1*min(data[obj[2]]),
                                    max(data[obj[2]])+0.1*max(data[obj[2]]))

                    ax.set_xlabel(x_label)
                    ax.set_ylabel(y_label)
                    ax.set_zlabel(z_label)
                    ax.legend(loc="lower left", fontsize=5)
                    ax.grid(True)

                # show plot for debugging
                if debug and figure_count == debug_figure_number-1 or debug_all:
                    plt.show()

                fig.suptitle(title, fontsize=16)
                plt.tight_layout()
                
                # save the figure as a page in the PDF
                pdf_pages.savefig(fig)
                plt.close(fig)

                figure_count += 1
                status_text = ">>> created figure {}: {}".format(figure_count, title)
                print(status_text)

    pdf_pages.close()


if __name__ == "__main__":
    # change the current working directory to the directory of this file
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # load the plot settings
    settings_file = "settings.yaml"
    with open(settings_file, 'r') as f:
        settings = yaml.load(f, Loader=yaml.FullLoader)

    mode = "manual single"
    # mode = "manual range"
    # mode = "auto"

    if mode == "manual single":
        # get the log number from the user
        log_num = input("Enter the logging number: ")
        log_str = f"log{log_num}"

        # decode binary log data
        path = os.path.join(settings["data_dir"], log_str)
        data_usd = cfusdlog.decode(path)

        # create the figures
        print("...creating figures")
        create_figures(data_usd, settings, log_str)
        print("...done creating figures")

    # TODO: manual range not tested (problems with the name of the logging file may arise bc of suffix "_2" for example)
    if mode == "manual range":
        # get the log number from the user
        log_num_first = input("Enter the first logging number: ")
        log_num_last = input("Enter the last logging number: ")

        for log_num in range(int(log_num_first), int(log_num_last) + 1):
            log_str = f"log{log_num}"

            # decode binary log data
            path = os.path.join(settings["data_dir"], log_str)
            data_usd = cfusdlog.decode(path)

            # create the figures
            print("...creating figures")
            create_figures(data_usd, settings, log_str)
            print("...done creating figures")

    elif mode == "auto":
        # automatically scan and process the logs
        # (1) do a scan of the directories logs, info, and reports to see what logs have not been plotted yet
        logs_dir = settings["data_dir"]
        info_dir = settings["info_dir"]
        reports_dir = settings["output_dir"]

        processed_logs = []
        for root, _, files in os.walk(reports_dir):
            for filename in files:
                if filename.startswith("log") and filename.endswith(".pdf"):
                    # Extract the log number from the PDF filename
                    log_str = filename.strip(".pdf")
                    processed_logs.append(log_str)
                    
        non_processed_logs = []
        for root, _, files in os.walk(logs_dir):
            for filename in files:
                if filename.startswith("log"):
                    if filename not in processed_logs:
                        non_processed_logs.append(filename)    

        # (2) plot them all
        if len(non_processed_logs) == 0:
            print("No logs to process")
            sys.exit(0)

        print("====================================")
        print("...logs to process:")
        non_processed_logs.sort()
        for log_str in non_processed_logs:
            print(log_str)
        print("====================================")
        ans = input("Proceed? [y/n]: ")
        if ans == "n":
            print("Exiting...")
            sys.exit(0)
        print("...processing logs")
        print("====================================")

        count = 0
        for log_str in non_processed_logs:
            # decode binary log data
            path = os.path.join(settings["data_dir"], log_str)
            data_usd = cfusdlog.decode(path)

            # create the figures
            print("...creating figures")
            create_figures(data_usd, settings, log_str)
            print("...done creating figures")

            count += 1

        print("====================================")
        print(f"Processed {count} logs")