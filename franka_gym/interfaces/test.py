class A(object):
    def __del__(self):
        print("Del!")

A()