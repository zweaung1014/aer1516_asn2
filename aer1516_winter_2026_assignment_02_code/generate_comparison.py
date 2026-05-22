"""
AER1516 Assignment 2 - Question 3
Comparison script: RRT vs RRT* performance on random Dubins planning instances.

This script generates comparison.pdf containing:
1. Plot of the obstacle environment
2. Bar chart comparing average path costs

Usage: python generate_comparison.py
"""

import random
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

# Import the problem class and planners
from dubins_path_problem import RRT_dubins_problem, get_path

# ============================================================
# CONFIGURATION - Edit these values as needed
# ============================================================
SEED = 42                  # Random seed for reproducibility
NUM_RUNS = 100             # Number of random instances
MAX_ITER = 1000            # Maximum iterations per planner run

# ============================================================
# ENVIRONMENT DESIGN
# ============================================================
# Map boundaries
MAP_AREA = [0.0, 20.0, 0.0, 20.0]  # [min_x, max_x, min_y, max_y]

# Obstacle layout: Staggered rows of obstacles creating suboptimal paths
OBSTACLE_LIST = [
    # Row 1 (y ~ 7)
    (5, 7, 2.0),
    (10, 7, 2.0),
    (15, 7, 2.0),
    # Row 2 (y ~ 13)
    (3, 13, 2.0),
    (10, 13, 2.0),
    (17, 13, 2.0),
]


def is_collision_free_point(x, y, obstacle_list, margin=0.5):
    """Check if a point (x, y) is not inside any obstacle (with margin)."""
    for ox, oy, size in obstacle_list:
        dist = math.hypot(x - ox, y - oy)
        if dist < size + margin:
            return False
    return True


def generate_random_pose(map_area, obstacle_list):
    """Generate a random collision-free pose (x, y, yaw) within map bounds."""
    min_x, max_x, min_y, max_y = map_area
    margin = 1.0  # Keep away from map edges
    
    for _ in range(1000):  # Max attempts
        x = random.uniform(min_x + margin, max_x - margin)
        y = random.uniform(min_y + margin, max_y - margin)
        if is_collision_free_point(x, y, obstacle_list):
            yaw = random.uniform(-math.pi, math.pi)
            return [x, y, yaw]
    
    # Fallback (should rarely happen)
    return [min_x + margin, min_y + margin, 0.0]


def run_comparison():
    """Run RRT and RRT* on multiple random instances and collect results."""
    print(f"Running comparison with seed={SEED}, num_runs={NUM_RUNS}")
    print(f"Max iterations per run: {MAX_ITER}")
    print("-" * 60)
    
    # Set random seed for reproducibility
    random.seed(SEED)
    np.random.seed(SEED)
    
    # Results storage
    rrt_costs = []
    rrt_star_costs = []
    rrt_failures = 0
    rrt_star_failures = 0
    
    for i in range(NUM_RUNS):
        # Generate random start and goal poses
        start = generate_random_pose(MAP_AREA, OBSTACLE_LIST)
        goal = generate_random_pose(MAP_AREA, OBSTACLE_LIST)
        
        # Ensure start and goal are not too close
        while math.hypot(start[0] - goal[0], start[1] - goal[1]) < 5.0:
            goal = generate_random_pose(MAP_AREA, OBSTACLE_LIST)
        
        # Run RRT
        random.seed(SEED + i)  # Reset seed for each run to ensure reproducibility
        np.random.seed(SEED + i)
        rrt_problem = RRT_dubins_problem(
            start=start,
            goal=goal,
            obstacle_list=OBSTACLE_LIST,
            map_area=MAP_AREA,
            max_iter=MAX_ITER
        )
        rrt_path = rrt_problem.rrt_planning(display_map=False)
        
        if rrt_path and len(rrt_path) > 0:
            rrt_cost = rrt_path[-1].cost
            rrt_costs.append(rrt_cost)
        else:
            rrt_failures += 1
        
        # Run RRT* with same start/goal
        random.seed(SEED + i)  # Reset seed for fair comparison
        np.random.seed(SEED + i)
        rrt_star_problem = RRT_dubins_problem(
            start=start,
            goal=goal,
            obstacle_list=OBSTACLE_LIST,
            map_area=MAP_AREA,
            max_iter=MAX_ITER
        )
        rrt_star_path = rrt_star_problem.rrt_star_planning(display_map=False)
        
        if rrt_star_path and len(rrt_star_path) > 0:
            rrt_star_cost = rrt_star_path[-1].cost
            rrt_star_costs.append(rrt_star_cost)
        else:
            rrt_star_failures += 1
        
        # Progress update
        if (i + 1) % 10 == 0:
            print(f"Completed {i + 1}/{NUM_RUNS} runs...")
    
    print("-" * 60)
    print(f"RRT:  {len(rrt_costs)} successes, {rrt_failures} failures")
    print(f"RRT*: {len(rrt_star_costs)} successes, {rrt_star_failures} failures")
    
    return rrt_costs, rrt_star_costs


def plot_environment(ax):
    """Plot the obstacle environment on the given axes."""
    ax.set_xlim(MAP_AREA[0] - 1, MAP_AREA[1] + 1)
    ax.set_ylim(MAP_AREA[2] - 1, MAP_AREA[3] + 1)
    
    # Plot obstacles
    for ox, oy, size in OBSTACLE_LIST:
        circle = plt.Circle((ox, oy), size, color='blue', alpha=0.7)
        ax.add_patch(circle)
    
    # Plot map boundaries
    ax.plot([MAP_AREA[0], MAP_AREA[1], MAP_AREA[1], MAP_AREA[0], MAP_AREA[0]],
            [MAP_AREA[2], MAP_AREA[2], MAP_AREA[3], MAP_AREA[3], MAP_AREA[2]],
            'k-', linewidth=2)
    
    # Add sample start and goal for illustration
    random.seed(SEED)
    sample_start = generate_random_pose(MAP_AREA, OBSTACLE_LIST)
    sample_goal = generate_random_pose(MAP_AREA, OBSTACLE_LIST)
    
    ax.plot(sample_start[0], sample_start[1], 'go', markersize=10, label='Sample Start')
    ax.plot(sample_goal[0], sample_goal[1], 'r*', markersize=12, label='Sample Goal')
    
    # Draw arrows showing orientation
    arrow_len = 1.0
    ax.arrow(sample_start[0], sample_start[1], 
             arrow_len * math.cos(sample_start[2]), arrow_len * math.sin(sample_start[2]),
             head_width=0.3, head_length=0.2, fc='green', ec='green')
    ax.arrow(sample_goal[0], sample_goal[1],
             arrow_len * math.cos(sample_goal[2]), arrow_len * math.sin(sample_goal[2]),
             head_width=0.3, head_length=0.2, fc='red', ec='red')
    
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_title('Obstacle Environment for RRT vs RRT* Comparison')
    ax.legend(loc='upper right')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)


def plot_bar_chart(ax, rrt_costs, rrt_star_costs):
    """Plot bar chart comparing average path costs."""
    # Calculate statistics
    rrt_avg = np.mean(rrt_costs) if rrt_costs else 0
    rrt_star_avg = np.mean(rrt_star_costs) if rrt_star_costs else 0
    rrt_std = np.std(rrt_costs) if rrt_costs else 0
    rrt_star_std = np.std(rrt_star_costs) if rrt_star_costs else 0
    
    # Create bar chart
    algorithms = ['RRT', 'RRT*']
    avg_costs = [rrt_avg, rrt_star_avg]
    std_costs = [rrt_std, rrt_star_std]
    colors = ['#ff7f0e', '#2ca02c']  # Orange for RRT, Green for RRT*
    
    bars = ax.bar(algorithms, avg_costs, yerr=std_costs, capsize=10, 
                  color=colors, edgecolor='black', linewidth=1.5)
    
    # Add value labels on bars
    for bar, avg, std in zip(bars, avg_costs, std_costs):
        height = bar.get_height()
        ax.annotate(f'{avg:.2f}\n(±{std:.2f})',
                    xy=(bar.get_x() + bar.get_width() / 2, height),
                    xytext=(0, 5),
                    textcoords="offset points",
                    ha='center', va='bottom', fontsize=11, fontweight='bold')
    
    ax.set_ylabel('Average Path Cost', fontsize=12)
    ax.set_title(f'RRT vs RRT* Average Path Cost Comparison\n({NUM_RUNS} Random Instances)', fontsize=12)
    ax.grid(True, axis='y', alpha=0.3)
    
    # Calculate and display improvement
    if rrt_avg > 0:
        improvement = (rrt_avg - rrt_star_avg) / rrt_avg * 100
        ax.text(0.5, 0.95, f'RRT* improvement: {improvement:.1f}%',
                transform=ax.transAxes, ha='center', va='top',
                fontsize=11, style='italic',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    return rrt_avg, rrt_star_avg


def generate_pdf(rrt_costs, rrt_star_costs):
    """Generate the comparison.pdf file."""
    output_file = 'comparison.pdf'
    
    with PdfPages(output_file) as pdf:
        # Page 1: Environment plot
        fig1, ax1 = plt.subplots(figsize=(10, 10))
        plot_environment(ax1)
        
        # Add seed note to environment plot
        fig1.text(0.5, 0.02, f'Random Seed: {SEED}', ha='center', fontsize=10,
                  style='italic', bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.5))
        
        plt.tight_layout()
        pdf.savefig(fig1, bbox_inches='tight')
        plt.close(fig1)
        
        # Page 2: Bar chart
        fig2, ax2 = plt.subplots(figsize=(10, 8))
        rrt_avg, rrt_star_avg = plot_bar_chart(ax2, rrt_costs, rrt_star_costs)
        
        # Add detailed statistics and seed note
        stats_text = (
            f"Statistics Summary:\n"
            f"  RRT:  n={len(rrt_costs)}, avg={np.mean(rrt_costs):.2f}, std={np.std(rrt_costs):.2f}\n"
            f"  RRT*: n={len(rrt_star_costs)}, avg={np.mean(rrt_star_costs):.2f}, std={np.std(rrt_star_costs):.2f}\n"
            f"\nRandom Seed: {SEED}"
        )
        fig2.text(0.5, 0.02, stats_text, ha='center', fontsize=9,
                  family='monospace',
                  bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.5))
        
        plt.tight_layout()
        plt.subplots_adjust(bottom=0.18)
        pdf.savefig(fig2, bbox_inches='tight')
        plt.close(fig2)
    
    print(f"\nGenerated: {output_file}")
    print(f"  - Page 1: Obstacle environment")
    print(f"  - Page 2: Cost comparison bar chart")


def main():
    print("=" * 60)
    print("AER1516 Assignment 2 - Question 3")
    print("RRT vs RRT* Performance Comparison")
    print("=" * 60)
    
    # Run the comparison
    rrt_costs, rrt_star_costs = run_comparison()
    
    # Print summary statistics
    if rrt_costs and rrt_star_costs:
        print("\n" + "=" * 60)
        print("RESULTS SUMMARY")
        print("=" * 60)
        print(f"RRT  Average Cost: {np.mean(rrt_costs):.2f} (std: {np.std(rrt_costs):.2f})")
        print(f"RRT* Average Cost: {np.mean(rrt_star_costs):.2f} (std: {np.std(rrt_star_costs):.2f})")
        improvement = (np.mean(rrt_costs) - np.mean(rrt_star_costs)) / np.mean(rrt_costs) * 100
        print(f"RRT* Improvement:  {improvement:.1f}%")
        print("=" * 60)
    
    # Generate PDF
    generate_pdf(rrt_costs, rrt_star_costs)
    
    print("\nDone! Submit comparison.pdf with your assignment.")


if __name__ == "__main__":
    main()
