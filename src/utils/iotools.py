class IOTools:
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