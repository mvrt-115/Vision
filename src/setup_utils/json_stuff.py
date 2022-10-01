import json

class JsonTools:
    def __init__(self):
        pass

    def read(self, filename):
        file = open(filename, "r+")
        text = file.read()
        file.close()
        return text

    def write(self, filename, text):
        file = open(filename, "w")
        file.writelines(text)
        file.close()

    def getJsonFrom(self, filename, key):
        text = self.read(filename)
        return json.loads(text)[key]

    def writeJson(self, filename, text):
        self.write(filename, json.dumps(text))