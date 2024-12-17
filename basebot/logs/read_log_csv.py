import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from io import StringIO
from sys import argv
import scipy as sp
from matplotlib.patches import Circle  # Import Circle from patches


# Check if a file was passed as an argument
is_custom_file = False
if len(argv) > 1:
    log_file = argv[1]
    if not os.path.isfile(log_file):
        raise FileNotFoundError(f"File '{log_file}' not found.")
    print(f"Reading log file '{log_file}'")
    is_custom_file = True



########## Get latest log file ##########

if len(argv) <= 1:
    # Get a list of all .log files in the current directory
    log_files = [f for f in os.listdir('.') if f.endswith('.log')]

    # Check if any log files exist
    if not log_files:
        raise FileNotFoundError("No .log files found in the current directory.")

    # Sort the files by date (latest first)
    log_files.sort(key=lambda x: os.path.getmtime(x), reverse=True)

    # Get the name of the most recent file
    log_file = log_files[0]



########## Read CSV ##########

# Read the file, skipping lines that start with '%'
with open(log_file, 'r') as file:
    lines = [line for line in file if not line.lstrip().startswith('%')]

# Read the log file
# See logger.h for format of the file
# Convert the filtered lines into a DataFrame
csv = pd.read_csv(StringIO(''.join(lines)), skipinitialspace=True)
# Convert to dictionary of numpy arrays
dd = {key: np.array(value) for key, value in csv.to_dict(orient='list').items()}

base_name = os.path.splitext(log_file)[0]

print(f"The CSV file has {csv.shape[0]} rows and {csv.shape[1]} columns.")



########## Plot ##########

plt.figure(figsize=(10, 12))

# Velocity
plt.subplot(3, 1, 1)
plt.plot(dd['time'], dd['vel_ref'], label='Velocity Reference')
plt.plot(dd['time'], dd['velocity'], label='Velocity')
# plt.plot(dd['time'], dd['right_vel'], '--', label='Right Velocity')
# plt.plot(dd['time'], dd['left_vel'], '--', label='Left Velocity')
plt.ylabel('Speed [m/s]')
plt.grid(True)
plt.legend()

# Heading

### Lacchezzo perchè mi sono dimenticato di loggare is_using_heading_controller  ###
v = 0.5;
mask_lacchezzo = (dd['time'] < (1.4/v)*1000) | (dd['time'] > (3.9/v)*1000)
dd['heading_ref'][~mask_lacchezzo] = np.nan
if base_name == 'best_real_life':
    mask_lacchezzo2 = dd['time'] > (4.8/v)*1000
    dd['heading_ref'][mask_lacchezzo2] = np.nan
###

if base_name != 'best_real_life':
    mask = dd['is_using_heading_controller'].astype(bool)
    dd['heading_ref'][~mask] = np.nan

# # Remove heading reference when not using heading controller
# dd['heading_ref'][dd['is_using_heading_controller']] = np.nan
dd['heading_ref'] = np.rad2deg(dd['heading_ref'])
dd['heading'] = np.rad2deg(dd['heading'])

plt.subplot(3, 1, 2)
plt.plot(dd['time'], dd['heading_ref'], label='Heading Reference')
plt.plot(dd['time'], dd['heading'], label='Heading')
# Integrate gyro to get heading
# h = sp.integrate.cumulative_trapezoid(dd['gyroZ'], dd['time']/1000, initial=0)
# plt.plot(dd['time'], h, '--', label='$\\int Gyro$', color='lightblue')
plt.ylabel('Angle [deg]')
plt.yticks(np.arange(-45, 360, 45))
plt.grid(True)
plt.legend()

# Turnrate
plt.subplot(3, 1, 3)
plt.plot(dd['time'], dd['turnrate_ref'], label='Turnrate Reference')
plt.plot(dd['time'], dd['turnrate'], label='Turnrate')
# plt.plot(dd['time'], np.deg2rad(dd['gyroZ']), '--', label='Gyro', color='lightblue')
plt.ylabel('Angular velocity [rad/s]')
plt.xlabel('Time [ms]')
plt.grid(True)
plt.legend()

plt.tight_layout()
if is_custom_file:
    plt.savefig(f'images/{base_name}_data.png')
else:
    plt.show()



########## PLOT POSE ##########

plt.figure(figsize=(8, 8))
plt.plot(dd['x'][0], dd['y'][0], 'o', color='blue', markersize=10, label='Start', linewidth=2)
plt.plot(dd['x'][-1], dd['y'][-1], 'x', color='red', markersize=10, label='End', linewidth=2)
plt.plot(dd['x'], dd['y'], 'b-', linewidth=2, label='Trajectory')

# Loop through the points to draw circles with orientation lines
circle_radius = 0.1
curr = 0
for i in range(0, len(dd['time'])):
    dist = np.sqrt((dd['x'][i] - dd['x'][i-1])**2 + (dd['y'][i] - dd['y'][i-1])**2)

    if dd['distance'][i] == curr:
        curr += 0.5

        # Create a circle at the position (x, y)
        # circle = Circle((dd['x'][i], dd['y'][i]), circle_radius, color='blue', fill=False)
        # plt.gca().add_artist(circle)

        # Draw the orientation line
        h = np.deg2rad(dd['heading'][i])
        plt.quiver(dd['x'][i], dd['y'][i], np.cos(h), np.sin(h), width=0.005, headwidth=5, color='red')
        # plt.plot([dd['x'][i], dd['x'][i] + np.cos(h) * circle_radius], 
        #      [dd['y'][i], dd['y'][i] + np.sin(h) * circle_radius], 
        #      color='red', lw=2)

# Uncomment this line to plot heading vectors
plt.quiver(dd['x'][-1], dd['y'][-1], np.cos(dd['heading'][-1]), np.sin(dd['heading'][-1]), width=0.005, headwidth=5, color='red')

plt.xlabel('X Position (meters)')
plt.ylabel('Y Position (meters)')
plt.title('Robot Trajectory')
plt.legend()
plt.axis('equal')
plt.grid(True)

if is_custom_file:
    plt.savefig(f'images/{base_name}_pose.png')
else:
    plt.show()

