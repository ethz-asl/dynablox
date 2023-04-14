#!/bin/bash

# Functions.
function run_experiments() {
  # Run all datasets
  for scene in "${scenes[@]}" 
  do
    for seq in "${sequences[@]}" 
    do
      # Make sure path exists.
      output=$output_path/doals/$scene/sequence_$seq
      if [ -d "$output" ]; then
        echo "Directory '$output' exists."
      else
        echo "Directory '$output' did not exist, was created."
        mkdir -p $output
      fi 
      for int in "${drift_intensities[@]}" 
      do
        local_rollouts=("${drift_rollouts[@]}")   
        if [ "$int" == "none" ]; then
        local_rollouts=(1)
        rollout="doals/$scene/sequence_$seq/none.csv"
        fi
        
        for r in "${local_rollouts[@]}" 
        do
          # Configure drift rollout.
          if [ "$int" != "none" ]; then
            rollout="doals/$scene/sequence_$seq/${int}_$r.csv"
          fi
          
          # Run experiment.
          echo "Started running experiment: $scene, sequence_$seq, drift: $int, rollout: $r."
            roslaunch drift_simulation generate_drift_rollout.launch bag_file:=$data_path/$scene/sequence_$seq/bag.bag drift_intensity:=$int output_drifted_file_name:=$output_path/$rollout player_rate:=$player_rate
          echo "Finished running experiment: $scene, sequence_$seq, drift: $int, rollout: $r."
        done
      done
    done
  done
}



# General params.
data_path="/media/lukas/T7/Datasets/DOALS"
player_rate="3"

# Data to run.
scenes=(niederdorf shopville station hauptgebaeude)
sequences=(1 2)
drift_intensities=(moderate) # none light moderate strong severe
drift_rollouts=(1 2 3)
output_path="/home/lukas/motion_ws/src/dynablox/drift_simulation/config/rollouts"

# ====== Run Experiments ======
run_experiments
