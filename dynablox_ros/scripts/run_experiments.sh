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
          
          # Check skipping.
          if [ "$skip_existing_runs" = true ] ; then
            if [ -d "$output_path/$nametag" ] ; then
              echo "$nametag already exists, skipping."
              continue
            fi
          fi

          # Run experiment.
          echo "Started running experiment: $scene, sequence_$seq, drift: $int, rollout: $r."
            roslaunch lidar_motion_detection_ros run.launch sequence:=sequence_$seq bag_file:=$data_path/$scene/sequence_$seq/bag.bag player_rate:=$player_rate drift_simulation_rollout:=$rollout evaluate:=true ground_truth_file:=$data_path/$scene/sequence_$seq/indices.csv config_file:=$config_file visualize:=false eval_output_path:=$output_path/$nametag counter_to_reset:=$counter_to_reset
          echo "Finished running experiment: $scene, sequence_$seq, drift: $int, rollout: $r."
        done
      done
    done
  done
}



# General params.
data_path="/home/lukas/data/DOALS"
skip_existing_runs=true
player_rate="0.8"

# Data to run.
scenes=(hauptgebaeude niederdorf station shopville) # hauptgebaeude niederdorf station shopville
sequences=(1 2) # 1 2
drift_intensities=(none) # none light moderate strong severe)
drift_rollouts=(1 2 3)  # 1 2 3

# Method to run.
config_file="motion_detector/doals.yaml"
counter_to_reset=150  # 10000, 150, 50, 40, 15
output_path="/mnt/c/Users/DerFu/Dokumente/motion_detection/data/"

# ====== Run Experiments ======
output_path="/mnt/c/Users/DerFu/Dokumente/motion_detection/data/20m_inf_eval"
config_file="motion_detector/1.yaml"
run_experiments

# Full experiments.
#drift_intensities=(none light moderate strong severe)
#vals=(150 50 40 15 10000)
#scenes=(hauptgebaeude niederdorf station shopville)
#for v in "${vals[@]}" 
#do
#  output_path="/media/lukas/T7/data/ours/doals_drift/$v"
#  counter_to_reset=$v
#  run_experiments
#done

