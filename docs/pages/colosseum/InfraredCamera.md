This is a tutorial for generating simulated thermal infrared (IR) images using AirSim and the AirSim Africa environment. 

Pre-compiled Africa Environment can be downloaded from the Releases tab of this Github repo: 
[Windows Pre-compiled binary](https://github.com/Microsoft/AirSim/releases/tag/v1.2.1)

To generate your own data, you may use two python files: [create_ir_segmentation_map.py](https://github.com/Microsoft/AirSim/tree/main/PythonClient//computer_vision/create_ir_segmentation_map.py) and 
[capture_ir_segmentation.py](https://github.com/Microsoft/AirSim/tree/main/PythonClient//computer_vision/capture_ir_segmentation.py).

[create_ir_segmentation_map.py](https://github.com/Microsoft/AirSim/tree/main/PythonClient//computer_vision/create_ir_segmentation_map.py) uses temperature, emissivity, and camera response information to estimate the thermal digital count that could be expected for the objects in the environment, and then reassigns the segmentation IDs in AirSim to match these digital counts. It should be run before starting to capture thermal IR data. Otherwise, digital counts in the IR images will be incorrect. The camera response, temperature, and emissivity data are all included for the Africa environment.

[capture_ir_segmentation.py](https://github.com/Microsoft/AirSim/tree/main/PythonClient//computer_vision/capture_ir_segmentation.py) is run after the segmentation IDs have been reassigned. It tracks objects of interest and records the infrared and scene images from the multirotor. It uses Computer Vision mode.

Finally, the details about how temperatures were estimated for plants and animals in the Africa environment, etc. can be found in this paper:

    @inproceedings{bondi2018airsim,
      title={AirSim-W: A Simulation Environment for Wildlife Conservation with UAVs},
      author={Bondi, Elizabeth and Dey, Debadeepta and Kapoor, Ashish and Piavis, Jim and Shah, Shital and Fang, Fei and Dilkina, Bistra and Hannaford, Robert and Iyer, Arvind and Joppa, Lucas and others},
      booktitle={Proceedings of the 1st ACM SIGCAS Conference on Computing and Sustainable Societies},
      pages={40},
      year={2018},
      organization={ACM}
    }
nb
