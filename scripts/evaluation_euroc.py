#!/usr/bin/python
# -*- coding: UTF-8 -*- 

import os
import math
import numpy as np
import shutil
import re

def read_tum_file(file_path):
  traj = []
  with open(file_path, "r") as f:
    for line in f.readlines():
      line = line.strip('\n').split(' ')
      pose = [float(l) for l in line]
      traj.append(pose)
  return traj

def MakeDir(nd):
  if not os.path.exists(nd):
    os.mkdir(nd)



# traj_gt_dir = "/media/code/ubuntu_files/airvo/experiments/traj_gt/oivio"
# saving_root = "/media/data/datasets/oivio/results/air_slam/"


# traj_gt_dir = "/media/code/ubuntu_files/airvo/experiments/traj_gt/uma"
# saving_root = "/media/data/datasets/uma/selected_seq/results/air_slam"


traj_gt_dir = "/workspace/dataset/ASL"
saving_root = "/workspace/eval/euroc"


# traj_gt_dir = "/media/bssd/datasets/tartanair/ground_truth/abandonedfactory"
# saving_root= "/media/code/ubuntu_files/airvio/experiments/results/tartanair/abandonedfactory"


# traj_gt_dir = "/media/data/datasets/euroc/dark_euroc/ground_truth"
# saving_root = "/media/data/datasets/euroc/dark_euroc/results/air_slam"



version = 0

MakeDir(saving_root)

traj_filename = "trajectory_v" + str(version) + ".txt"
sum_save_name = "rmse_v" + str(version) + ".txt"

map_root = os.path.join(saving_root, "maps")
evo_save_root = os.path.join(saving_root, "evo")
MakeDir(evo_save_root)

tmp_root = os.path.join(evo_save_root, "tmp")
MakeDir(tmp_root)
eva_root = os.path.join(tmp_root, "evaluation")
MakeDir(eva_root)
eva_seq_root = os.path.join(tmp_root, "seq")
MakeDir(eva_seq_root)

sequences = os.listdir(map_root)
print(sequences)
for sequence in sequences:
  # regu exp only for MH dataset
  gt_path = os.path.join(traj_gt_dir, (re.findall(r"(MH_\d+_.+)_.*", sequence)[0]+"/state_groundtruth_estimate0/data.csv"))
  result_root = os.path.join(map_root, sequence)
  result_path = os.path.join(result_root, traj_filename)
  
  eva_seq_file = sequence + ".zip"
  eva_seq_path = os.path.join(eva_seq_root, eva_seq_file)
  print(gt_path)
  print("eva_seq_path = {}".format(eva_seq_path))
  os.system("evo_ape euroc {} {} -as --save_results {}".format(gt_path, result_path, eva_seq_path))

table_file = os.path.join(evo_save_root, sum_save_name)
os.system("evo_res {}/*.zip -p --use_filenames --save_table {}".format(eva_seq_root, table_file))

shutil.rmtree(tmp_root)
