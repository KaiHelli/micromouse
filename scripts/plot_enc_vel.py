# Import necessary libraries
import pandas as pd
import matplotlib.pyplot as plt

# Load the linear and rotational velocity profiles
velocities = pd.read_csv('/Users/kai.helli/MPLABXProjects/micromouseLC.X/data/enc_velocity_comp.csv')

# Display a preview of the data (optional, for verification)
print(velocities.head())

# Filter out continuous zero velocity segments, keeping only the first and last zero
#linear_df['non_zero_segment'] = (linear_df['measuredLinVel'] != 0).astype(int).diff().ne(0).cumsum()
#linear_df = linear_df.groupby('non_zero_segment').apply(lambda g: g if g['measuredLinVel'].iloc[0] != 0 else g.iloc[[0, -1]]).reset_index(drop=True).drop(columns=['non_zero_segment'])

#rotational_df['non_zero_segment'] = (rotational_df['measuredAngVel'] != 0).astype(int).diff().ne(0).cumsum()
#rotational_df = rotational_df.groupby('non_zero_segment').apply(lambda g: g if g['measuredAngVel'].iloc[0] != 0 else g.iloc[[0, -1]]).reset_index(drop=True).drop(columns=['non_zero_segment'])

velocities['timestamp'] -= velocities['timestamp'][0]


# Plot the linear velocity profile
plt.figure(figsize=(7, 4.5))

# Linear velocity plot
plt.plot(velocities['timestamp'], velocities['vLeft'], label='PLL Estimated Velocity', color='orange', linewidth=1.5)
plt.plot(velocities['timestamp'], velocities['vRight'], label='Naive Estimated Velocity', color='darkorange', alpha=0.5, linewidth=1.5, linestyle='--')
plt.title('Wheel Velocity Estimation - PLL vs Naive')
plt.xlabel('Timestamp [s]')
plt.ylabel('Speed [mm/s]')
plt.legend()
plt.grid(True)
plt.savefig(f"enc_velocities_comparison.png", dpi=300)

plt.tight_layout()
plt.show()

