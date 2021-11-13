class JsonParser:
    def __init__(self, json_file):
        self.json_file = json_file
        
    def getNPCInitialPosition(self):
        with open(self.json_file) as json_data:
            d = json.load(json_data)
            return d['npc_initial_position']