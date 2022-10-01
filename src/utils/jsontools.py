import json
from iotools import IOTools

class JsonTools:
    def __init__(self):
        self.tools = IOTools()

    def getJsonVal(self, filename, key):
        text = self.tools.read(filename)
        return json.loads(text)[key]

    def writeJson(self, filename, text):
        self.tools.write(filename, json.dumps(text))