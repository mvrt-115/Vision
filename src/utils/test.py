from iotools import IOTools
from jsontools import JsonTools

new = IOTools() 
new.write('test.txt', '{ "hello" : "lilja" }')
new.read('test.txt')

blah = JsonTools()
print(blah.getJsonVal('test.txt', 'hello'))