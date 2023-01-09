#!/bin/bash

# Functions.
function run_experiments() {
  # Make sure output directory exists.
  if [ -d "$output_path" ]; then
    echo "Directory '$output_path' exists."
  else
    echo "Directory '$output_path' did not exist, was created."
    mkdir -p $output_path
  fi 

  # Run all datasets
  for scene in "${scenes[@]}" 
  do
    for seq in "${sequences[@]}" 
    do
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
          nametag="${scene}_${seq}_$int"
          if [ "$int" != "none" ]; then
            rollout="doals/$scene/sequence_$seq/${int}_$r.csv"
            nametag="${nametag}_$r"
          fi

          # Run experiment.
          echo "Started running experiment: $scene, sequence_$seq, drift: $int, rollout: $r."
            roslaunch lidar_motion_detection_ros run.launch sequence:=sequence_$seq bag_file:=$data_path/$scene/sequence_$seq/bag.bag player_rate:=$player_rate drift_simulation_rollout:=$rollout evaluate:=true ground_truth_file:=$data_path/$scene/sequence_$seq/indices.csv config_file:=$config_file visualize:=false eval_output_path:=$output_path/$nametag
          echo "Finished running experiment: $scene, sequence_$seq, drift: $int, rollout: $r."
        done
      done
    done
  done
}



# General params.
data_path="/home/voliro/data"
player_rate="0.2"

# Data to run.
scenes=(hauptgebaeude niederdorf shopville station) # hauptgebaeude niederdorf station shopville
sequences=(1 2)
drift_intensities=(none) # none light moderate strong severe)
drift_rollouts=(1 2 3)

# Method to run.
config_file="motion_detector/doals.yaml"
output_path="/media/lukas/T7/data/doals_nodrift_inf"

# ====== Run Experiments ======
run_experiments
