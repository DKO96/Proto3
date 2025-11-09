import csv
import serial 
import matplotlib.pyplot as plt

def read_lines_forever(ser):
  """Read lines continuously with error handling"""
  while True:
    try:
      line = ser.readline()
      if line:
        text = line.decode('utf-8', errors='ignore').strip()
        if text:  # Skip empty lines
          yield text
    except KeyboardInterrupt:
      break
    except Exception as e:
      print(f"Read error: {e}")
      break

def get_velocity(position):
  velocity = []

  for i in range(1, len(position) - 1):
    delta = position[i] - position[i-1] / 0.0001
    velocity.append(delta)
  return velocity


def main():
  position = []
  ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

  try:
    for line in read_lines_forever(ser):
        step = int(line)

        print(f"Got: {step}")
        position.append(step)

  except KeyboardInterrupt:
    print("\nStopped by Ctrl+C")

  finally:
    ser.close()
  
  # Save plot
  if position:

    with open('position.csv', 'w', newline='') as f:
      writer = csv.writer(f)
      writer.writerow(position)

    # velocity = get_velocity(position)

    # Create both plots in one window
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))

    # Plot velocity
    # ax1.plot(range(len(velocity)), velocity, label="Velocity", linewidth=2)
    # ax1.set_xlabel("Timestep")
    # ax1.set_ylabel("Velocity")
    # ax1.set_title("Velocity plot")
    # ax1.grid()
    # ax1.legend()

    # Plot position
    ax2.plot(range(len(position)), position, label="Position", linewidth=2)
    ax2.set_xlabel("Timestep")
    ax2.set_ylabel("Position")
    ax2.set_title("Position plot")
    ax2.grid()
    ax2.legend()

    plt.tight_layout()
    plt.show()
  else:
    print("No data collected")


if __name__ == "__main__":
  main()