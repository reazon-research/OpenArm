# Copyright 2025 Reazon Holdings, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import csv
import matplotlib.pyplot as plt

events = []
values = []
key = ['Event', 'Value']  # Titles you're looking for

# Open the CSV file
with open('event_recorder_save_1.csv', newline='') as csvfile:
    csvreader = csv.reader(csvfile)
    
    # Read the header to get the indices of the desired columns
    header = next(csvreader)  # Read the first row (header)
    
    # Strip spaces from header values to avoid mismatches
    header = [col.strip() for col in header]

    # Initialize a dictionary to hold column indices
    column_indices = {}

    # Find the index of each key in the header dynamically
    for k in key:
        if k in header:
            column_indices[k] = header.index(k)
        else:
            print(f"Title '{k}' not found in the header.")
            column_indices[k] = None

    # Extract the values under those titles if they exist
    for row in csvreader:
        if column_indices[key[0]] is not None:
            events.append(row[column_indices[key[0]]])

        if column_indices[key[1]] is not None:
            value_str = row[column_indices[key[1]]]

            # Split the string if it contains two hexadecimal values
            parts = value_str.split()

            if len(parts) == 2:
                # Check if both parts are valid hexadecimal
                try:
                    # Try to parse both parts as hexadecimal values
                    first_value = int(parts[0], 16)
                    second_value = int(parts[1], 16)

                    # Convert the second hexadecimal value to a float
                    second_value_float = float(second_value)
                    values.append(second_value_float)
                except ValueError:
                    # In case of an invalid hexadecimal value
                    print(f"Invalid hexadecimal value: {value_str}")
            else:
                print(f"Value '{value_str}' doesn't contain exactly two hexadecimal parts.")

# Truncate the lists if the lengths don't match
min_length = min(len(events), len(values))
events = events[:min_length]
values = values[:min_length]

# Downsampling: Keep only every 10th point (or adjust the step as needed)
downsample_rate = 100  # Adjust this number based on how much downsampling you want
events = events[::downsample_rate]
values = values[::downsample_rate]

# Plot the data
plt.figure(figsize=(10, 6))

# Scatter plot instead of line plot for faster rendering
plt.scatter(events, values, color='b', label='Event vs Velocity', s=10)  # s is the size of points

# Customize the plot
plt.xlabel('Event')
plt.ylabel('Velocity')
plt.title('Event vs Velocity Plot')
plt.xticks(rotation=45)  # Rotate x-axis labels if needed
plt.legend()

# Show the plot
plt.show()
