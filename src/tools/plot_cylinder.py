import pandas as pd
import matplotlib.pyplot as plt

def plot_xy_graph(csv_file):
    try:
        # Read the CSV file into a DataFrame
        df = pd.read_csv(csv_file)

        # Extract the first and second columns as x and y
        x = df.iloc[:, 0].values.tolist()
        y = df.iloc[:, 1].values.tolist()

        # Create the XY graph
        plt.figure(figsize=(8, 6))  # Adjust the figure size as needed
        plt.scatter(x, y, marker='o')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Scan')
        plt.grid(True)

        # Show the plot
        plt.show()
    except Exception as e:
        print(f"An error occurred: {e}")

# Usage example:
root_path = '/home/seb/ros2ws/rbtp2_ws/src/tp2sim'
csv_file = root_path + '/resource/cylinders_points.csv'  # Replace with the path to your CSV file
plot_xy_graph(csv_file)