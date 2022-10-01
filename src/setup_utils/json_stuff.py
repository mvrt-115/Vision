import json

class JsonTools:
    def __init__(self):
        print('init')

    def read(self, filename):
        file = open(filename, "r+")
        text = file.read()
        file.close()
        return text

    def write(self, filename, text):
        file = open(filename, "w")
        file.writelines(text)
        file.close()

tools = JsonTools()
tools.read('text.txt')
tools.write('text.txt', 'heheheh')