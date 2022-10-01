from json_stuff import JsonTools

tools = JsonTools()

x = {
    "fx" : 1000,
    "fy" : 103
}

tools.writeJson('test.txt', x)