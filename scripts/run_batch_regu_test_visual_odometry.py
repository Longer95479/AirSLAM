import os
import yaml

def MakeDir(nd):
  if not os.path.exists(nd):
    os.mkdir(nd)

def modify_yaml(file_path, key_paths, values):
    with open(file_path, 'r') as file:
        data = yaml.load(file, Loader=yaml.FullLoader)
    
    for key_path, value in zip(key_paths, values):
        expr = "data" + key_path + "=" + value 
        print(expr)
        exec(expr)  
            
    with open(file_path, 'w') as file:
        yaml.dump(data, file)


dataset_idx = 0
datasets = ["euroc_norm", "uma", "oivio", "tartanair", "euroc_dark"]
dataset = datasets[dataset_idx]

workspace = "/root/catkin_ws"

if "euroc_norm" in dataset:
  dataroot = "/workspace/dataset/ASL"
  saving_root = "/workspace/eval/euroc"
  launch_file = "vo_euroc.launch"
  yaml_config = "/root/catkin_ws/src/AirSLAM/configs/visual_odometry/vo_euroc_4batchrun.yaml"
elif "uma" in dataset:
  dataroot = "/media/data/datasets/uma/selected_seq/air_slam"
  saving_root = "/media/data/datasets/uma/selected_seq/results/air_slam"
  launch_file = "vo_uma_bumblebee.launch"
elif "oivio" in dataset:
  dataroot = "/media/data/datasets/oivio/selected_seq"
  saving_root = "/media/data/datasets/oivio/results/air_slam"
  launch_file = "vo_oivio.launch"
elif "tartanair" in dataset:
  dataroot = "/media/bssd/datasets/tartanair/euroc_style/with_time/abandonedfactory"
  saving_root = "/media/code/ubuntu_files/airvio/experiments/results/tartanair/abandonedfactory"
  launch_file = "vo_tartanair.launch"
if "euroc_dark" in dataset:
  dataroot = "/media/data/datasets/euroc/dark_euroc/sequences"
  saving_root = "/media/data/datasets/euroc/dark_euroc/results/air_slam"
  launch_file = "vo_euroc_dark.launch"
else:
  print("{} is not support".format(dataset_idx))


print(dataset_idx)
print(dataroot)

MakeDir(saving_root)
map_root = os.path.join(saving_root, "maps")
MakeDir(map_root)
# sequences = os.listdir(dataroot)
sequences = ["MH_01_easy", "MH_02_easy", "MH_03_medium", "MH_04_difficult", "MH_05_difficult"]
# sequences = ["MH_02_easy"]

# use point only
key_paths = ["['regularity_encoder']['use_regu']", "['regularity_encoder']['epsilon']"]
values = ["True", "0.0000000001"]
modify_yaml(yaml_config, key_paths, values)
for sequence in sequences:
  seq_dataroot = os.path.join(dataroot, sequence)
  seq_save_root = os.path.join(map_root, sequence+"_pts")
  MakeDir(seq_save_root)
  os.system("cd {} & roslaunch air_slam {} dataroot:={} saving_dir:={} config_path:={} visualization:=false".format(workspace, launch_file, seq_dataroot, seq_save_root, yaml_config))

# use lines (oringin version)
key_paths = ["['regularity_encoder']['use_regu']", "['regularity_encoder']['epsilon']"]
values = ["False", "0.0175"]
modify_yaml(yaml_config, key_paths, values)
for sequence in sequences:
  seq_dataroot = os.path.join(dataroot, sequence)
  seq_save_root = os.path.join(map_root, sequence+"_ori")
  MakeDir(seq_save_root)
  os.system("cd {} & roslaunch air_slam {} dataroot:={} saving_dir:={} config_path:={} visualization:=false".format(workspace, launch_file, seq_dataroot, seq_save_root, yaml_config))

# use regu
key_paths = ["['regularity_encoder']['use_regu']", "['regularity_encoder']['epsilon']"]
values = ["True", "0.0175"]
modify_yaml(yaml_config, key_paths, values)
for sequence in sequences:
  seq_dataroot = os.path.join(dataroot, sequence)
  seq_save_root = os.path.join(map_root, sequence+"_regu")
  MakeDir(seq_save_root)
  os.system("cd {} & roslaunch air_slam {} dataroot:={} saving_dir:={} config_path:={} visualization:=false".format(workspace, launch_file, seq_dataroot, seq_save_root, yaml_config))

