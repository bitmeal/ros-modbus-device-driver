
class DiscreteDecoder:
    def __init__(self, states):
        self.states = states
        self.pointer = 0
    
    def decode(self):
        self.pointer += 1
        return self.states[self.pointer - 1]
    
    def reset(self):
        self.pointer = 0

    def skip(self, count):
        self.pointer += count
