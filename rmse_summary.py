#!/usr/bin/env python
import os
import glob
import re
from collections import defaultdict

def get_last_value_from_file(file_path):
    last_line = None
    with open(file_path, 'r') as file:
        for line in file:
            if 'data: ' in line:
                last_line = line
    if last_line:
        return last_line.split('data: ')[1].strip()
    else:
        return '0'  # Return a default value if no valid data line was found

def sort_errors(errors):
    # Define the desired order
    order = {'heading': 0, 'position': 1, 'speed': 2}
    # Sort the errors based on the predefined order
    return sorted(errors, key=lambda e: order[e.split()[0]])

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))  # Get the directory of the current script
    log_dir = os.path.join(script_dir, 'logs')  # Path to the logs directory within the same directory as the script
    summary_file_name = 'a_summary_log.txt'
    summary_file_path = os.path.join(log_dir, summary_file_name)

    # Use a dictionary to group data by timestamp
    grouped_data = defaultdict(list)

    for file_path in glob.glob(f"{log_dir}/rms_*_error_log_*.txt"):
        # Extract error type and timestamp from the file name
        match = re.search(r'rms_(.*)_error_log_(\d{8}_\d{6})\.txt', os.path.basename(file_path))
        if match:
            error_type = match.group(1)
            timestamp = match.group(2)

            # Get the last value from the file
            last_value = get_last_value_from_file(file_path)
            # Group data by timestamp, with error_type as key for sorting later
            grouped_data[timestamp].append(f"{error_type} error: {last_value}")
        else:
            print(f"Unable to extract details from file name: {file_path}")

    # Write the grouped and sorted data to the summary file
    with open(summary_file_path, 'w') as summary_file:
        for timestamp, errors in sorted(grouped_data.items()):
            summary_file.write(f"<{timestamp}>\n")
            # Sort errors according to the predefined order
            sorted_errors = sort_errors(errors)
            for error in sorted_errors:
                summary_file.write(f"{error}\n")
            summary_file.write("\n")  # Add a newline for separation between groups

    # Change the file permissions to 777
    os.chmod(summary_file_path, 0o777)

    print("Script execution completed. Summary log file has been updated.")

if __name__ == "__main__":
    main()
