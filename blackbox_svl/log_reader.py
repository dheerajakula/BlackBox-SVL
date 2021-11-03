import json
import gzip

def deserialize_log_file(log_file_path):
    # read every alternate line as json
    final_data = []
    # open file
    with gzip.open(log_file_path, 'r') as f:
        # read every alternate line as json
        # return data
        data = []
        for line in f:
            if line.startswith(str.encode('{')):
                data.append(json.loads(line))
        intial_frame_tick = data[0]
        intial_time = intial_frame_tick['Time']

        for frame_tick in data:
            per_frame_data = [0.0 for i in range(11)]
            elapsed_time = frame_tick['Time'] - intial_time
            per_frame_data[0] = elapsed_time
            for vehicle in frame_tick['Data']:
                if vehicle['Id'] == 1:
                    ego_position = vehicle['Position']
                    per_frame_data[1] = ego_position['x']
                    per_frame_data[2] = ego_position['z']
                    ego_rotation = vehicle['Rotation']
                    per_frame_data[3] = ego_rotation['w']
                    ego_velocity = vehicle['LinearVelocity']
                    per_frame_data[4] = ego_velocity['x']
                if vehicle['Id'] > 2:
                    npc_position = vehicle['Position']
                    per_frame_data[5] = npc_position['x']
                    per_frame_data[6] = npc_position['z']
                    npc_rotation = vehicle['Rotation']
                    per_frame_data[7] = npc_rotation['w']
                    npc_velocity = vehicle['LinearVelocity']
                    per_frame_data[8] = npc_velocity['x']
            final_data.append(per_frame_data)
        return final_data


if __name__ == "__main__":
    #final_data = deserialize_log_file("C:\\Users\\dheer\\AppData\\LocalLow\\LGElectronics\\SVLSimulator\\simulation_log.json.gz")
    final_data = deserialize_log_file("/home/local/ASUAD/vakula1/.config/unity3d/LGElectronics/SVLSimulator/simulation_log.json.gz")
    print(final_data[0])