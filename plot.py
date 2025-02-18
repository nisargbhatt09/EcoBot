import matplotlib.pyplot as plt
import numpy as np

# Sample Data
path_numbers = [0, 1, 2, 3]
navigation_times = [4, 12, 18, 16]  # Example times for navigation
path_lengths = [0.01, 0.02, 0.03, 0.04]  # Example path lengths

objects = ["soupcan", "orange2", "tennis_ball"]
grasp_success_rate = [100, 100, 90]  # Success rates for each object

task_stages = ["picking", "placing", "total"]
task_execution_times = [30, 25, 100]  # Example times for task stages

# Create subplots
fig, axs = plt.subplots(2, 2, figsize=(15, 10))

# Plot 1: Navigation Time per Path
axs[0, 0].plot(path_numbers, navigation_times, marker='o')
axs[0, 0].set_title("Navigation Time per Path")
axs[0, 0].set_xlabel("Path Number")
axs[0, 0].set_ylabel("Time (seconds)")

# Plot 2: Grasp Success Rate by Object
axs[0, 1].bar(objects, grasp_success_rate)
axs[0, 1].set_title("Grasp Success Rate by Object")
axs[0, 1].set_xlabel("Objects")
axs[0, 1].set_ylabel("Success Rate (%)")

# Plot 3: Path Length per Navigation
axs[1, 0].plot(path_numbers, path_lengths, marker='o')
axs[1, 0].set_title("Path Length per Navigation")
axs[1, 0].set_xlabel("Path Number")
axs[1, 0].set_ylabel("Path Length (units)")

# Plot 4: Average Task Execution Times
axs[1, 1].bar(task_stages, task_execution_times)
axs[1, 1].set_title("Average Task Execution Times")
axs[1, 1].set_xlabel("Task Stages")
axs[1, 1].set_ylabel("Time (seconds)")

# Adjust layout and show
plt.tight_layout()
plt.show()