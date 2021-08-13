class data_type(object):
    def __init__(self):
        self.x = 0

class data_type2(data_type):
    def __init__(self):
        self.y = 0
        self.aa=self.show()
    
    def show(self):
        self.a = data_type()
        # print(self.a.x)
    def sh(self):
        print(self.a.x + 1)

a = data_type2()
a.sh()