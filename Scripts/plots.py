import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal

def main():
  # Read the CSV file
  df = pd.read_csv('position.csv', header=None)

  # Get the position data from the first row
  position_raw = df.iloc[0].values

  # Sampling parameters
  sampling_period = 100e-6  # 100 microseconds in seconds
  sampling_rate = 1 / sampling_period  # 10,000 Hz

  # Apply exponential moving average (EMA) low-pass filter to POSITION first
  # Alpha controls smoothing: lower = smoother, higher = more responsive
  # Typical range: 0.01 to 0.3
  alpha = 0.01  # Smoothing factor (adjust this for more/less smoothing)
  
  # Apply EMA filter to position
  position_filtered = np.zeros_like(position_raw, dtype=float)
  position_filtered[0] = position_raw[0]
  for i in range(1, len(position_raw)):
    position_filtered[i] = alpha * position_raw[i] + (1 - alpha) * position_filtered[i-1]

  # Create time array in seconds
  time = np.arange(len(position_filtered)) * sampling_period

  # Calculate velocity from FILTERED position using numerical differentiation
  velocity = np.diff(position_filtered) / sampling_period

  # Time array for velocity (one point shorter due to diff)
  time_velocity = time[:-1]

  # Create figure with dual y-axes
  fig, ax1 = plt.subplots(figsize=(14, 7))

  # Plot position on primary y-axis
  color1 = '#2E86AB'
  ax1.set_xlabel('Time (ms)', fontsize=12)
  ax1.set_ylabel('Position', fontsize=12, color=color1)
  line1 = ax1.plot(time * 1000, position_filtered, linewidth=2, color=color1, label='Position (filtered)')
  ax1.tick_params(axis='y', labelcolor=color1)
  ax1.grid(True, alpha=0.3)

  # Create secondary y-axis for velocity
  ax2 = ax1.twinx()
  color2 = '#A23B72'
  ax2.set_ylabel('Velocity (units/s)', fontsize=12, color=color2)
  line2 = ax2.plot(time_velocity * 1000, velocity, linewidth=2, color=color2, 
                   label='Velocity', alpha=0.8)
  ax2.tick_params(axis='y', labelcolor=color2)

  # Add title
  plt.title('Position and Velocity Over Time', fontsize=14, fontweight='bold', pad=20)

  # Combine legends
  lines = line1 + line2
  labels = [l.get_label() for l in lines]
  ax1.legend(lines, labels, loc='upper left', fontsize=10)

  plt.tight_layout()

  # Save the plot
  # plt.savefig('/mnt/user-data/outputs/position_velocity_combined.png', dpi=300, bbox_inches='tight')
  # print("Plot saved successfully!")

  # Display statistics
  print(f"\nPosition Statistics:")
  print(f"  Filter: Exponential Moving Average (alpha={alpha})")
  print(f"  Total samples: {len(position_filtered)}")
  print(f"  Time duration: {time[-1]*1000:.2f} ms")
  print(f"  Raw position - Min: {position_raw.min()}, Max: {position_raw.max()}")
  print(f"  Filtered position - Min: {position_filtered.min():.2f}, Max: {position_filtered.max():.2f}")

  print(f"\nVelocity Statistics (from filtered position):")
  print(f"  Min velocity: {velocity.min():.2f} units/s")
  print(f"  Max velocity: {velocity.max():.2f} units/s")
  print(f"  Average velocity: {velocity.mean():.2f} units/s")
  print(f"  Std deviation: {velocity.std():.2f} units/s")

  # Calculate velocity from raw position for comparison
  velocity_raw = np.diff(position_raw) / sampling_period
  print(f"\nVelocity Statistics (from raw position - for comparison):")
  print(f"  Min velocity: {velocity_raw.min():.2f} units/s")
  print(f"  Max velocity: {velocity_raw.max():.2f} units/s")
  print(f"  Std deviation: {velocity_raw.std():.2f} units/s")

  plt.show()



if __name__ == "__main__":
  main()