# Import necessary libraries
import pandas as pd
import matplotlib.pyplot as plt

# Load the linear and rotational velocity profiles
linear_df = pd.read_csv('/Users/kai.helli/MPLABXProjects/micromouseLC.X/data/vel_profile_lin_tuned.csv')
rotational_df = pd.read_csv('/Users/kai.helli/MPLABXProjects/micromouseLC.X/data/vel_profile_rot_tuned.csv')

# Display a preview of the data (optional, for verification)
print(linear_df.head())
print(rotational_df.head())

# Filter out continuous zero velocity segments, keeping only the first and last zero
#linear_df['non_zero_segment'] = (linear_df['measuredLinVel'] != 0).astype(int).diff().ne(0).cumsum()
#linear_df = linear_df.groupby('non_zero_segment').apply(lambda g: g if g['measuredLinVel'].iloc[0] != 0 else g.iloc[[0, -1]]).reset_index(drop=True).drop(columns=['non_zero_segment'])

#rotational_df['non_zero_segment'] = (rotational_df['measuredAngVel'] != 0).astype(int).diff().ne(0).cumsum()
#rotational_df = rotational_df.groupby('non_zero_segment').apply(lambda g: g if g['measuredAngVel'].iloc[0] != 0 else g.iloc[[0, -1]]).reset_index(drop=True).drop(columns=['non_zero_segment'])

linear_df['timestamp'] -= linear_df['timestamp'][0]
rotational_df['timestamp'] -= rotational_df['timestamp'][0]


# Plot the linear velocity profile
plt.figure(figsize=(7, 4.5))

# Linear velocity plot
plt.plot(linear_df['timestamp'], linear_df['idealLinearSpeed'], label='Ideal Linear Speed', color='orange', linewidth=1.5)
plt.plot(linear_df['timestamp'], linear_df['measuredLinVel'], label='Measured Linear Speed', color='darkorange', linewidth=1.5, linestyle='--')
plt.title('Linear Velocity Profile')
plt.xlabel('Timestamp [s]')
plt.ylabel('Speed [m/s]')
plt.legend(loc='upper right')
plt.grid(True)
plt.savefig(f"vel_profile_linear.png", dpi=300)

plt.tight_layout()
plt.show()


plt.figure(figsize=(7, 4.5))
# Rotational velocity plot
plt.plot(rotational_df['timestamp'], rotational_df['idealAngularSpeed'], label='Ideal Angular Speed', color='orange', linewidth=1.5)
plt.plot(rotational_df['timestamp'], rotational_df['measuredAngVel'], label='Measured Angular Speed', color='darkorange', linewidth=1.5, linestyle='--')
plt.title('Rotational Velocity Profile')
plt.xlabel('Timestamp [s]')
plt.ylabel('Angular Speed [rad/s]')
plt.legend(loc='upper right')
plt.grid(True)
plt.savefig(f"vel_profile_angular.png", dpi=300)

plt.tight_layout()
plt.show()