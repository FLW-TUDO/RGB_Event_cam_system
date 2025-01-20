# Create a class which stores vicon data  and has setter and getter methods to get the data

class DataClass:
    # Constructor to initialize attributes
    def __init__(self, data):
        self.data = data

    def get_rotation(self,t):
        # check if value corresponding to the time exists in data dictionary keys. If not then find the closed entry
        if str(t) not in self.data.keys():
            t = min(self.data.keys(), key=lambda x: abs(int(x) - int(str(t))))
        return self.data[str(t)]['rotation']


