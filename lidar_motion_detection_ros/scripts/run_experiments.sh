#!/bin/bash

# Functions.
function run_experiments() {
  # Make sure output directory exists.
  if [ -d "$data_dir" ]; then
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
        rollout="\"\""
        if [ "$drift_intensities" == "none" ]; then
        drift_rollouts=(1)
        fi
        for r in "${drift_rollouts[@]}" 
        do
          # Configure drift rollout.
          nametag="${scene}_${seq}_$int"
          if [ "$drift_intensities" != "none" ]; then
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
config_file="motion_detector/doals.yaml"
player_rate="0.2"
output_path="/media/lukas/T7/data/doals_nodrift"

# Data to run.
data_path="/media/lukas/T7/Datasets/DOALS"
scenes=(hauptgebaeude niederdorf shopville station)
sequences=(1 2)
drift_intensities=(none) # light medium severe)
drift_rollouts=(1 2 3)

# ====== Run Experiments ======
run_experiments
