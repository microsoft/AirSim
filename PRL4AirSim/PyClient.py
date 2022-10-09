import Utils as Utils
import DQNTrainer as DQNTrainer
import datetime
import time
import Simulation as Simulation
import argparse

if __name__ == "__main__":
    """
        Model Server port 29000
        UE Server port 29001
    """

    parser = argparse.ArgumentParser(description="PyClient",
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("UE_Port")
    parser.add_argument("UE_Address")
    parser.add_argument("storage_port")

    args = parser.parse_args()
    arguments = vars(args)

    trainer_ip_address = '127.0.0.1' #os.environ['BUFFER_SERVER_IP']
    #trainer_port = int(29000) #int(os.environ['BUFFER_SERVER_PORT'])
    storage_port = int(arguments["storage_port"])
    ue_ip_address = arguments["UE_Address"] #os.environ['UE_SERVER_IP']
    #ue_ip_address = str(arguments["IP_Address"])
    ue_port = int(arguments["UE_Port"]) #int(os.environ['UE_SERVER_PORT'])

    client, model_server = Utils.connectClient(trainer_ip_address=trainer_ip_address, ue_ip_address=ue_ip_address, trainer_port=storage_port, ue_port=ue_port)
    times = []

    ## Setup Environment
    image_shape = (2, 32, 32)

    now = datetime.datetime.now()
    current_time = now.strftime("%H:%M:%S")
    print("start time: ", current_time)


    agent = DQNTrainer.DQNTrainer(image_input_dims=Utils.getConfig()['state_space'],
                                  n_actions=Utils.getConfig()['action_space'],
                                  replayMemory_size=Utils.getConfig()['buffer_Size'],
                                  batch_size=Utils.getConfig()['batch_size'],
                                  learningRate=Utils.getConfig()['learning_rate'],
                                  discount_factor=Utils.getConfig()['discount_factor'],
                                  epsilon=1.0,
                                  replace_target_count_episode=Utils.getConfig()['replace_target_count_episode'])

    #print("loaded best model")
    #agent.load('{}/BestModelSaves/dqn.pth'.format(pathlib.Path().resolve()))

    run_name = now.strftime("%Y_%m_%d_%Hh%Mm%Ss")

    simulation = Simulation.Sim(image_shape=Utils.getConfig()['state_space'], num_drones=Utils.getConfig()['num_drones'])
    train = Utils.getConfig()['from_artifact'] == ''

    start = (time.perf_counter() / 3600)
    Utils.getModelServer().call("startSimulation")
    while simulation.episodes < Utils.getConfig()['max_episodes']:
        finished = simulation.tick(agent)

    end = datetime.datetime.now()
    current_time = end.strftime("%H:%M:%S")
    print("End time: ", current_time)
