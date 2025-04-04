import os
import yaml
import numpy as np

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
# sequences = ["MH_01_easy", "MH_02_easy", "MH_03_medium", "MH_04_difficult", "MH_05_difficult"]
sequences = ["MH_04_difficult"]

tau = [i for i in range(5,6)]
print("tau:", tau)

k_car = [i for i in range(3,12,2)]
print("k_car:", k_car)

k_epsi = [i for i in range(2,7,2)]
print ("k_epsi:" ,k_epsi)

params = []
names = []
for t in tau:
    for kc in k_car:
        for ke in k_epsi:
            t_s = f"{t}"
            kc_s = f"{kc*np.pi/180}"
            ke_s = f"{np.sin(ke*np.pi/180)}"
            params.append(["True", t_s, kc_s, ke_s])
            names.append("_tau"+t_s+"_kcar"+f"{kc}"+"_ke"+f"{ke}")

print(params)
print(names)


# use regu
key_paths = ["['regularity_encoder']['use_regu']", "['regularity_encoder']['tau']", "['regularity_encoder']['cardin_peak_thr']", "['regularity_encoder']['epsilon']"]
for sequence in sequences:
  for values, name in zip(params, names):
    modify_yaml(yaml_config, key_paths, values)
    seq_dataroot = os.path.join(dataroot, sequence)
    seq_save_root = os.path.join(map_root, sequence+name)
    MakeDir(seq_save_root)
    os.system("cd {} & roslaunch air_slam {} dataroot:={} saving_dir:={} config_path:={} visualization:=false".format(workspace, launch_file, seq_dataroot, seq_save_root, yaml_config))

