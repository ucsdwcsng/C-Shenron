# class path_config:
# 	path_csv = "/media/ehdd_8t1/RadarImaging/collab_radar_data/simulator_inputs/sumo_traffic_data/"
# 	path_ply = "/media/ehdd_8t1/RadarImaging/collab_radar_data/simulator_inputs/point_cloud_maps/"
# 	path_cad = "/media/ehdd_8t1/RadarImaging/collab_radar_data/simulator_inputs/car_cad_models/"

path_config = {
    "path_csv": "/media/ehdd_8t1/RadarImaging/collab_radar_data/simulator_inputs/sumo_traffic_data/",
    "path_ply": "/media/ehdd_8t1/RadarImaging/collab_radar_data/simulator_inputs/point_cloud_maps/",
    "path_cad": "/media/ehdd_8t1/RadarImaging/collab_radar_data/simulator_inputs/car_cad_models/",
    "path_ped": "/media/ehdd_8t1/RadarImaging/collab_radar_data/simulator_inputs/human_cad_models/"
    }

out_paths = {
    "path_tensors": "/media/ehdd_8t1/RadarImaging/collab_radar_data/radar_tensors/",
    "path_gt": "/media/ehdd_8t1/RadarImaging/collab_radar_data/ground_truth/",
    "path_images": "/media/ehdd_8t1/RadarImaging/collab_radar_data/radar_bev_images/"
    }

dataset_config = {
	"dataset_name": "downtown_SD_10thru_50count", #"ucsd_atkinson_1"
	"ply_name": "downtown_SD_10_7.ply", #"ucsd_atkinson_10_7.ply"
	"csv_name": "downtown_SD_10thru_50count_10Hz_with_cad_id"
}

ped_dataset_config = {
	"dataset_name": "test_11_08_person3", 
	"npy_name": "test_11_08_person3_segmented", 
}

out_config = {
	"save_dataset_name" : "downtown_SD_10thru_50count_1deg_10Hz"
}

debug_config = {
	"save_cropped_path" : "/media/ehdd_8t1/RadarImaging/collab_radar_data/debug/"
}

# class data_paths:
# 	path_tensors = "/media/ehdd_8t1/RadarImaging/collab_radar_data/radar_tensors/"
# 	path_gt = "/media/ehdd_8t1/RadarImaging/collab_radar_data/ground_truth/"
# 	path_images = "/media/ehdd_8t1/RadarImaging/collab_radar_data/images_no_white/"


