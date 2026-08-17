class Test:
    def __init__(self, name):
        self.name = name

    def greet(self):
        return f"Hello, {self.name}!"


    if __name__ == "__main__":
        test_instance = Test("World")
        test_instance1 = Test("World")
        print(test_instance.greet())
        print(test_instance1.greet())