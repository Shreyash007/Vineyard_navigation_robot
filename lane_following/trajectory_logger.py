import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_trajectory(df, output_filename='trajectory_comparison.png'):
    """
    Plots the desired vs. actual trajectory with an enhanced Y-axis view.
    """
    print("Generating trajectory comparison plot with enhanced Y-axis...")
    plt.style.use('seaborn-v0_8-whitegrid')
    
    fig, ax = plt.subplots(figsize=(12, 7)) # Adjusted figsize for a wider plot
    
    try:
        # Plotting the data
        ax.plot(df['desired_x'], df['desired_y'], 'r--', label='Desired Path (from Perception)', linewidth=2.5, dashes=(4, 4))
        ax.plot(df['actual_x'], df['actual_y'], 'b-', label='Actual Path (Robot Odometry)', linewidth=3)
        ax.plot(df['actual_x'].iloc[0], df['actual_y'].iloc[0], 'o', color='green', markersize=12, label='Start', zorder=10)
        ax.plot(df['actual_x'].iloc[-1], df['actual_y'].iloc[-1], 'o', color='magenta', markersize=12, label='End', zorder=10)
        
        # --- FIXES FOR Y-AXIS VISIBILITY ---
        
        # 1. Remove equal aspect ratio to allow Y-axis to stretch
        # ax.set_aspect('equal', adjustable='box') # This line is now removed/commented out
        
        # 2. Manually set Y-axis limits to emphasize the deviation
        y_min_actual = df['actual_y'].min()
        y_max_actual = df['actual_y'].max()
        y_min_desired = df['desired_y'].min()
        y_max_desired = df['desired_y'].max()
        
        # Find the overall min and max across both paths
        overall_y_min = min(y_min_actual, y_min_desired)
        overall_y_max = max(y_max_actual, y_max_desired)
        
        # Add a buffer (e.g., 20% of the range) for better visualization
        y_range = overall_y_max - overall_y_min
        buffer = y_range * 0.2
        
        ax.set_ylim(-4.0, 4.0)

        # --- General Layout Improvements ---
        ax.set_title('Robot Trajectory vs. Desired Path from perception', fontsize=18, pad=20)
        ax.set_xlabel('X Position (meters)', fontsize=14)
        ax.set_ylabel('Y Position (meters)', fontsize=14)
        ax.legend(loc='best', fontsize=12, frameon=True, shadow=True)
        ax.tick_params(axis='both', which='major', labelsize=12)
        ax.grid(True)
        
        plt.tight_layout()
        
        plt.savefig(output_filename, dpi=300, bbox_inches='tight')
        print(f"  -> Trajectory plot saved to {output_filename}")
        plt.show()
    
    finally:
        plt.close(fig)

# --- The plot_error_over_time function and the main execution block remain unchanged ---

def plot_error_over_time(df, output_filename='tracking_error.png'):
    """
    Plots the tracking error over time with improved layout.
    """
    print("Generating tracking error plot...")
    plt.style.use('seaborn-v0_8-whitegrid')
    
    fig, ax = plt.subplots(figsize=(12, 6))

    try:
        df['time_elapsed'] = df['timestamp'] - df['timestamp'].iloc[0]
        ax.plot(df['time_elapsed'], df['tracking_error'], 'g-', label='Tracking Error')
        
        mean_error = df['tracking_error'].mean()
        ax.axhline(mean_error, color='purple', linestyle='--', label=f'Mean Error: {mean_error:.3f} m')

        ax.set_title('Tracking Error Over Time', fontsize=18, pad=20)
        ax.set_xlabel('Time (seconds)', fontsize=14)
        ax.set_ylabel('Error (meters)', fontsize=14)
        ax.tick_params(axis='both', which='major', labelsize=12)
        ax.legend(loc='best', fontsize=12, frameon=True, shadow=True)
        ax.grid(True)

        plt.tight_layout()
        
        plt.savefig(output_filename, dpi=300, bbox_inches='tight')
        print(f"  -> Error plot saved to {output_filename}")
        plt.show()

    finally:
        plt.close(fig)

if __name__ == '__main__':
    log_file = 'trajectory_log_1.csv'
    
    try:
        df = pd.read_csv(log_file)
        print(f"Successfully loaded '{log_file}' with {len(df)} data points.")
    except FileNotFoundError:
        print(f"Error: The file '{log_file}' was not found.")
        exit()
    except Exception as e:
        print(f"An error occurred while reading the CSV: {e}")
        exit()

    if not df.empty:
        try:
            df['tracking_error'] = pd.to_numeric(df['tracking_error'], errors='coerce')
            df.dropna(subset=['tracking_error'], inplace=True)

            mean_absolute_error = df['tracking_error'].mean()
            root_mean_square_error = np.sqrt((df['tracking_error']**2).mean())
            max_error = df['tracking_error'].max()

            print("\n--- Performance Metrics ---")
            print(f"Mean Absolute Error (MAE): {mean_absolute_error:.4f} meters")
            print(f"Root Mean Square Error (RMSE): {root_mean_square_error:.4f} meters")
            print(f"Maximum Tracking Error: {max_error:.4f} meters")
            print("---------------------------\n")

            plot_trajectory(df)
            plot_error_over_time(df)

        except (KeyError, Exception) as e:
            print(f"An error occurred during plotting or calculation: {e}")
    else:
        print("Log file is empty. No data to plot.")