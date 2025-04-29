#!/bin/bash

source /root/ros1_ws/devel/setup.bash

num_iterations=3  # Change as needed
sequences=(
           "cloud_avia_mid_w25_dynxtrz_e5" \
           "cloud_avia_mid_w35_dynxtrz_e5" \
           "cloud_avia_mid_w45_dynxtrz_e5" \
           "cloud_avia_mid_w55_dynxtrz_e5" \
           "cloud_avia_mid_w65_dynxtrz_e5" \
           "cloud_avia_mid_w75_dynxtrz_e5" \
           "cloud_avia_mid_w85_dynxtrz_e5" \
           "cloud_avia_mid_w95_dynxtrz_e5")

for seq in "${sequences[@]}"; do
    sequence="/root/ros1_ws/src/gptr/scripts/$seq.bag"
    echo "Processing sequence: $sequence"
    for ((i=0; i<num_iterations; i++)); do
        echo "Running iteration $i..."
        log_path="/root/ros1_ws/src/I2EKF-LO/sim_exp/ctemlo/$seq/try_$i"
        mkdir -p $log_path
        roslaunch cte_mlo mapping_liocath.launch log_path:=$log_path bag_file:="$sequence"
    done
done