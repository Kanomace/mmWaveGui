# @file        real_data
# @type        python
# @author      Kano
# @date        2024-03-18
# @brief       Used to read mmWave radar data via serial port
from gui_parser import *

# Define input and output file paths
input_file = 'originData.txt'
output_file = 'ProcessedData.txt'


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


def write_output_data(file_path, parsed_data):
    # Write parsed data to the output file
    with open(file_path, 'w') as file:
        for frame_data in parsed_data:
            # Write the parsed result to the file
            file.write(str(frame_data) + '\n')


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # Parse the input data
    parsed_data = parse_input_data(input_file)

    # Write the parsed results to the output file
    write_output_data(output_file, parsed_data)

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
