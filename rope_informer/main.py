# @file        real_data
# @type        python
# @author      Kano
# @date        2024-03-18
# @brief       Used to read mmWave radar data via serial port
from gui_parser import *
import pandas as pd
import numpy as np

# Define input and output file paths
input_file = 'pHistBytes_1.txt'
output_file = 'ProcessedData.xlsx'  # Change output format to xlsx


def parse_input_data(file_path):
    # Read data from the input file
    with open(file_path, 'r') as file:
        data = file.read()

    # Remove line breaks and split data into frames
    frames = data.split('\n')

    # Parse data frame by frame
    parsed_data = []
    for frame in frames:
        if len(frame) > 0:
            # Convert frame data from hex string to byte stream
            frame_data = bytes.fromhex(frame)

            # Call the parsing function to decode the frame data
            parsed_frame = parseStandardFrame(frame_data)

            # Append the parsed result to the list
            parsed_data.append(parsed_frame)

    return parsed_data


def extract_point_cloud(parsed_data):
    # Extract point cloud data from parsed frames
    point_cloud = []
    for frame in parsed_data:
        point_cloud.extend(frame['pointCloud'])
    return point_cloud


def write_output_data(file_path, point_cloud):
    # Create a DataFrame object
    df = pd.DataFrame(
        point_cloud,
        columns=['X', 'Y', 'Z', 'Doppler', 'SNR', 'Noise', 'Track index']
    )

    # Write the DataFrame to an Excel file
    df.to_excel(file_path, index=False)


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # Parse the input data
    parsed_data = parse_input_data(input_file)

    # Extract point cloud data
    point_cloud = extract_point_cloud(parsed_data)

    # Write the parsed results to the output file
    write_output_data(output_file, point_cloud)
