

class VariableSet():

    def __init__(self, key='', amount=1, values=[]):
        if len(values) == amount:
            for x, y in zip(range(1, amount+1), values):
                name = '{}{}'.format(key, x)
                exec("%s=%s" % (name, y))
        else:
            for x in range(1, amount+1):
                name = '{}{}'.format(key, x)
                exec("%s=%s" % (name, x))
