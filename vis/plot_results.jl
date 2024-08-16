using Pkg

Pkg.activate(".")
using HIPPO

Pkg.activate("vis")
using CairoMakie

# Data
planners = ["hippo_unified", "hippo_reactive", "greedy", "map"]
find_ratios = [0.62, 0.22, 0.28, 0.22]
avg_rewards = [4.362726121376905, 0.2888531792446503, 3.20168999449372, 1.2932582145530496]

# Create the plot
fig = Figure(size = (800, 600))
ax = Axis(fig[1, 1], 
    xlabel = "Find Ratio", 
    ylabel = "Average Reward",
    title = "Find Ratio vs Average Reward for Different Planners")

# Plot the points
scatter!(ax, find_ratios, avg_rewards, markersize = 15)

# Add labels for each point
for (i, planner) in enumerate(planners)
    text!(ax, find_ratios[i], avg_rewards[i], text = planner, 
          align = (:left, :bottom), offset = (5, 5))
end

# Display the plot
display(fig)

# Optionally, save the plot
save("find_ratio_vs_avg_reward.png", fig)