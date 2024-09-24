import pickle
import pandas as pd
import os
import numpy as np
from synchronization_experiments.nodes.data_collector import AgentData



def flatten_agent(agent):
    print(agent.action)
    return {
        'action': ','.join(map(str, agent.action)),
        'action_type': ','.join(map(str, agent.action_type)),
        'action_start': ','.join(map(str, agent.action_start)),
        'collab_start': ','.join(map(str, agent.collab_start))
    }

def unflatten_agent(data):
    action=data['action'].split(',')
    action_type=data['action_type'].split(',')
    action_start=str(data['action_start']).split(',')
    action_start=[float(s) for s in action_start]
    collab_start=str(data['collab_start']).split(',')
    collab_start=[float(s) for s in collab_start]
    
    
    agent_data=AgentData()
    agent_data.action=action
    agent_data.action_type=action_type
    agent_data.action_start=action_start
    agent_data.collab_start=collab_start
    return agent_data

def pkl_to_csv(pkl_file, csv_file):
    with open(pkl_file, 'rb') as file:
        data = pickle.load(file)
    
    flattened_data = {k: flatten_agent(v) for k, v in data.items()}
    df = pd.DataFrame.from_dict(flattened_data, orient='index').reset_index().rename(columns={'index': 'agent'})

    df.to_csv(csv_file, index=False)
    print(f"Pickle file converted to CSV and saved as {csv_file}")

def csv_to_pkl(csv_file, pkl_file):
    df = pd.read_csv(csv_file)
    data = {row['agent']: unflatten_agent(row) for _, row in df.iterrows()}

    with open(pkl_file, 'wb') as file:
        pickle.dump(data, file)
    
    print(f"CSV file converted to Pickle and saved as {pkl_file}")

def main():
    user_input = input("Enter 0 to convert from PKL to CSV, or 1 to convert from CSV to PKL: ")

    pkl_file = '/home/davide/git_thesis/LTL_CANOPIES_Synchronization_master_thesis/ros2_ws/src/synchronization_experiments/results/paper/90_agents_mod.pkl'
    csv_file = '/home/davide/git_thesis/LTL_CANOPIES_Synchronization_master_thesis/ros2_ws/src/synchronization_experiments/results/paper/90_agents_mod.csv'

    
    if user_input == '0':      
        if os.path.isfile(pkl_file):
            pkl_to_csv(pkl_file, csv_file)
        else:
            print(f"The file {pkl_file} does not exist.")
    elif user_input == '1':        
        if os.path.isfile(csv_file):
            csv_to_pkl(csv_file, pkl_file)
        else:
            print(f"The file {csv_file} does not exist.")
    else:
        print("Invalid input. Please enter 0 or 1.")

if __name__ == "__main__":
    main()
