#!/bin/bash

# Base directories
base_output_dir="./outputs/"
base_error_dir="./errors/"
log_dir="./logs/"
scenario_path="/path/to/your/scenario"  # Update this with your actual scenario path

# Create directories if they don't exist
mkdir -p "$base_output_dir" "$base_error_dir" "$log_dir"

# Fixed parameters
sync_interval=20
simulation_end=7200  # 1 hour simulation
enable_logging=true

# Arrays of parameters to test
num_agents=(1 2 3 4 5)
k_values=(1 3 5)
epsilon_values=(0.95 0.99 1.0)
reroute_portions=(0.5 0.7 1.0)

# Iterate through all combinations
for agents in "${num_agents[@]}"; do
    for k in "${k_values[@]}"; do
        for epsilon in "${epsilon_values[@]}"; do
            for reroute in "${reroute_portions[@]}"; do
                # Create a unique identifier for this configuration
                config_id="agents${agents}_k${k}_eps${epsilon}_reroute${reroute}"
                
                # Create output directories for this configuration
                config_output_dir="${base_output_dir}${config_id}/"
                config_error_dir="${base_error_dir}${config_id}/"
                mkdir -p "$config_output_dir" "$config_error_dir"
                
                # Submit the job
                sbatch <<EOT
#!/bin/bash
#SBATCH --nodes=1
#SBATCH --ntasks=$((agents + 1))
#SBATCH --cpus-per-task=1
#SBATCH --mem-per-cpu=16G
#SBATCH --partition=cpu
#SBATCH --job-name=mas_planner_${config_id}
#SBATCH --output=${log_dir}mas_planner_${config_id}_%j.out
#SBATCH --error=${log_dir}mas_planner_${config_id}_%j.err

# Load required modules
ml foss/2021a

# Set up output files for each agent (including master)
for i in $(seq 0 $agents); do
    agent_output="${config_output_dir}/agent${i}.out"
    agent_error="${config_error_dir}/agent${i}.err"
    touch "$agent_output" "$agent_error"
done

# Run the simulation with the current configuration using MPI
srun --cpu-bind=cores --mpi=pmix ./build/mas_planner \
    --agents $agents \
    --path "$scenario_path" \
    --sync $sync_interval \
    --tripinfo "tripinfo_${config_id}.xml" \
    --edgedata "edgedata_${config_id}.xml" \
    --end $simulation_end \
    --epsilon $epsilon \
    --k $k \
    --reroute $reroute \
    --logging > "${config_output_dir}/master.out" 2> "${config_error_dir}/master.err"

EOT
            done
        done
    done
done 